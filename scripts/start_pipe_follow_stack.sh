#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${ROS_WS:-$ROOT_DIR/ros2_ws}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$ROOT_DIR/ardupilot}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/log/autostart}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"

ARDUSUB_SERIAL0="${ARDUSUB_SERIAL0:-udp:127.0.0.1:14550}"
ARDUSUB_SERIAL1="${ARDUSUB_SERIAL1:-/dev/ttyACM0}"
ARDUSUB_START_DELAY_S="${ARDUSUB_START_DELAY_S:-2}"
DEVICE_WAIT_TIMEOUT_S="${DEVICE_WAIT_TIMEOUT_S:-30}"

ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-$ROS_WS/install/setup.bash}"
ROS_LAUNCH_PACKAGE="${ROS_LAUNCH_PACKAGE:-rov_pipe_tracker}"
ROS_LAUNCH_FILE="${ROS_LAUNCH_FILE:-pipe_follow_cv_container.launch.py}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
ROS_AUTOSTART_LOG="${ROS_AUTOSTART_LOG:-pipe_follow_cv.log}"
ROS_LAUNCH_ARGS="${ROS_LAUNCH_ARGS:-}"

ARDUSUB_PID=""
ROS_PID=""

log()
{
  printf '[%(%Y-%m-%d %H:%M:%S)T] %s\n' -1 "$*"
}

wait_for_device()
{
  local device="$1"
  local timeout_s="$2"

  if [[ "$device" != /dev/* ]]; then
    return 0
  fi

  local waited=0
  while [[ ! -e "$device" ]]; do
    if (( waited >= timeout_s )); then
      log "device not found after ${timeout_s}s: $device"
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
}

start_ardusub()
{
  cd "$ARDUPILOT_DIR"
  wait_for_device "$ARDUSUB_SERIAL1" "$DEVICE_WAIT_TIMEOUT_S"

  if (( EUID == 0 )); then
    exec ./build/subrov/bin/ardusub \
      --serial0 "$ARDUSUB_SERIAL0" \
      --serial1 "$ARDUSUB_SERIAL1"
  fi

  exec sudo ./build/subrov/bin/ardusub \
    --serial0 "$ARDUSUB_SERIAL0" \
    --serial1 "$ARDUSUB_SERIAL1"
}

start_ros_launch()
{
  cd "$ROS_WS"
  mkdir -p "$ROS_LOG_DIR"
  export ROS_LOG_DIR
  export HOME="${HOME:-/home/sunrise}"
  set +u
  # shellcheck disable=SC1090
  source "$ROS_DISTRO_SETUP"
  # shellcheck disable=SC1090
  source "$ROS_WS_SETUP"
  set -u
  export ROS_DOMAIN_ID
  export ROS_LOCALHOST_ONLY
  local extra_args=()
  if [[ -n "$ROS_LAUNCH_ARGS" ]]; then
    read -r -a extra_args <<< "$ROS_LAUNCH_ARGS"
  fi
  exec ros2 launch "$ROS_LAUNCH_PACKAGE" "$ROS_LAUNCH_FILE" "${extra_args[@]}" "$@"
}

cleanup()
{
  trap - EXIT INT TERM
  log "stopping pipe follow stack"
  if [[ -n "$ROS_PID" ]]; then
    kill -TERM "$ROS_PID" 2>/dev/null || true
  fi
  if [[ -n "$ARDUSUB_PID" ]]; then
    kill -TERM "$ARDUSUB_PID" 2>/dev/null || true
  fi
  wait "$ROS_PID" 2>/dev/null || true
  wait "$ARDUSUB_PID" 2>/dev/null || true
}

main()
{
  mkdir -p "$LOG_DIR"

  if [[ ! -x "$ARDUPILOT_DIR/build/subrov/bin/ardusub" ]]; then
    log "missing executable: $ARDUPILOT_DIR/build/subrov/bin/ardusub"
    return 1
  fi
  if [[ ! -f "$ROS_DISTRO_SETUP" ]]; then
    log "missing ROS distro setup: $ROS_DISTRO_SETUP"
    return 1
  fi
  if [[ ! -f "$ROS_WS_SETUP" ]]; then
    log "missing ROS workspace setup: $ROS_WS_SETUP"
    return 1
  fi

  trap cleanup EXIT INT TERM

  log "starting ArduSub: serial0=$ARDUSUB_SERIAL0 serial1=$ARDUSUB_SERIAL1"
  start_ardusub >"$LOG_DIR/ardusub.log" 2>&1 &
  ARDUSUB_PID=$!

  sleep "$ARDUSUB_START_DELAY_S"

  log "starting ROS launch: $ROS_LAUNCH_PACKAGE $ROS_LAUNCH_FILE $*"
  start_ros_launch "$@" >"$LOG_DIR/$ROS_AUTOSTART_LOG" 2>&1 &
  ROS_PID=$!

  set +e
  wait -n "$ARDUSUB_PID" "$ROS_PID"
  local status=$?
  set -e
  log "one process exited, status=$status"
  return "$status"
}

main "$@"

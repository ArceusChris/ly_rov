#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="${SERVICE_NAME:-rov-pipe-follow.service}"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
START_SCRIPT="$ROOT_DIR/scripts/start_pipe_follow_stack.sh"
ROS_LAUNCH_FILE_VALUE="${ROS_LAUNCH_FILE:-pipe_follow_cv_container.launch.py}"
ROS_AUTOSTART_LOG_VALUE="${ROS_AUTOSTART_LOG:-pipe_follow_cv.log}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-20}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"
ROS_LAUNCH_ARGS_VALUE="${ROS_LAUNCH_ARGS:-}"

if [[ ! -x "$START_SCRIPT" ]]; then
  chmod +x "$START_SCRIPT"
fi

sudo tee "$SERVICE_PATH" >/dev/null <<SERVICE
[Unit]
Description=ROV ArduSub and pipe follow CV stack
Wants=network-online.target
After=network-online.target systemd-udev-settle.service

[Service]
Type=simple
WorkingDirectory=$ROOT_DIR
ExecStart=$START_SCRIPT
Restart=on-failure
RestartSec=5
TimeoutStopSec=20
KillSignal=SIGTERM
Environment=RCUTILS_LOGGING_BUFFERED_STREAM=1
Environment=ROS_LAUNCH_FILE=$ROS_LAUNCH_FILE_VALUE
Environment=ROS_AUTOSTART_LOG=$ROS_AUTOSTART_LOG_VALUE
Environment=ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE
Environment=ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY_VALUE
Environment=ROS_LAUNCH_ARGS=$ROS_LAUNCH_ARGS_VALUE

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"

if [[ "${START_NOW:-1}" == "1" ]]; then
  sudo systemctl restart "$SERVICE_NAME"
fi

cat <<EOF
Installed $SERVICE_NAME

Status:
  sudo systemctl status $SERVICE_NAME

Logs:
  sudo journalctl -u $SERVICE_NAME -f
  tail -f $ROOT_DIR/log/autostart/ardusub.log
  tail -f $ROOT_DIR/log/autostart/$ROS_AUTOSTART_LOG_VALUE

Disable:
  sudo systemctl disable --now $SERVICE_NAME
EOF

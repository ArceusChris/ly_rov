# LY ROV Vision Pipeline

ROS 2 vision pipeline for an ROV:

1. capture camera images through V4L2,
2. run UDCP dehazing,
3. run YOLOv8 segmentation on the dehazed image,
4. use the segmentation mask to follow an underwater pipe,
5. send motion commands to a slave controller over MAVLink UDP.

The camera is assumed to be mounted vertically and looking downward.

## Repository Layout

```text
ros2_ws/src/camera_driver        V4L2 camera component, publishes bgr8 images
ros2_ws/src/dehaze               UDCP dehaze component and camera+dehaze launch
ros2_ws/src/dehaze_segmentation  YOLOv8-seg launch/config wrapper
ros2_ws/src/rov_pipe_tracker     Pipe tracking node and full pipeline launch
hobot_dnn                        D-Robotics DNN example dependency
mavlink                          MAVLink submodule used to generate C headers
ardupilot                        ArduPilot source/submodule
```

## Submodules

Initialize submodules before building:

```bash
git submodule update --init --recursive
```

`rov_pipe_tracker` generates MAVLink C headers from the `mavlink` submodule during CMake configure/build, so `mavlink/pymavlink` must also be initialized.

## Build

```bash
cd ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The YOLOv8 segmentation node reuses `dnn_node_example` and its dependencies. Make sure the D-Robotics runtime, `ai_msgs`, `dnn_node`, model files, and OpenCV/cv_bridge dependencies are available on the target system.

Default YOLOv8-seg model path:

```text
/opt/hobot/model/x5/basic/yolov8_seg_640x640_nv12.bin
```

Edit `ros2_ws/src/dehaze_segmentation/config/yolov8segworkconfig.json` if your platform or model path differs.

## Run

Camera + dehaze only:

```bash
ros2 launch dehaze camera_dehaze_container.launch.py \
  device:=/dev/video0 width:=320 height:=240 fps:=30
```

Camera + dehaze + YOLOv8 segmentation:

```bash
ros2 launch dehaze_segmentation dehaze_yolov8seg_container.launch.py \
  device:=/dev/video0 width:=640 height:=480 fps:=30
```

Full pipe-following pipeline:

```bash
ros2 launch rov_pipe_tracker pipe_follow_container.launch.py \
  device:=/dev/video0 \
  target_ip:=192.168.2.2 \
  target_port:=14550
```

OpenCV pipe segmentation with video Web UI and manual-control Web UI:

```bash
ros2 launch rov_pipe_tracker pipe_follow_cv_container.launch.py \
  device:=/dev/video0 \
  target_port:=14550
```

Default ports:

```text
8080  video, pipe-mask overlay monitor, and automatic command log
8081  manual motion control
```

Pipe tracker only, if segmentation is already running:

```bash
ros2 launch rov_pipe_tracker pipe_tracker.launch.py \
  mask_topic:=hobot_dnn_segmentation \
  target_ip:=192.168.2.2 \
  target_port:=14550
```

## Topics

```text
image_raw               Raw BGR image from camera_driver
image_dehazed           Dehazed BGR image from dehaze
hobot_dnn_segmentation  ai_msgs/msg/PerceptionTargets segmentation output
```

All provided full-pipeline launches put the camera, dehaze, segmentation, and tracker components in one ROS 2 component container with `use_intra_process_comms` enabled.

## Pipe Tracker Parameters

Important parameters:

```text
target_ip           MAVLink UDP target IP
target_port         MAVLink UDP target port, default 14550
mask_topic          Segmentation result topic
mask_label          Label to track; -1 means any non-zero mask pixel
min_pixels          Minimum valid pipe-mask pixels
desired_angle_deg   Desired pipe direction in image coordinates, default 90
forward_axis        MAVLink MANUAL_CONTROL x axis, -1000..1000
z_axis              MAVLink MANUAL_CONTROL z axis, 0..1000, default 500
lateral_gain        Gain from horizontal image error to MANUAL_CONTROL y
yaw_gain            Gain from pipe angle error to MANUAL_CONTROL r
lateral_sign        Flip lateral direction without changing code
yaw_sign            Flip yaw direction without changing code
stale_timeout_s     Send neutral command when mask data is stale
```

For a downward-facing camera, start with low `forward_axis`, test with thrusters disabled or constrained, then adjust `lateral_sign`, `yaw_sign`, and `desired_angle_deg` on the real vehicle.

## MAVLink Output

`rov_pipe_tracker` sends:

```text
HEARTBEAT       once per second
MANUAL_CONTROL  at command_rate_hz
```

When no valid mask is available, it sends neutral control:

```text
x=0 y=0 z=500 r=0
```

## Manual Control Web UI

The manual-control UI is available at:

```text
http://<ROV_IP>:8081
```

It publishes `manual_control_command` with this layout:

```text
[x, y, z, r, override_enabled]
```

`x/y/r` use `-1000..1000`, `z` uses `0..1000`, and `z=500` is neutral.
When manual override is enabled, manual commands take priority over automatic pipe
following. Press `RELEASE AUTO` to hand control back to automatic pipe following.

## Safety

Test MAVLink output and sign conventions with the vehicle restrained or thrusters disabled first. The tracker is intentionally minimal: it follows the largest valid segmentation mask and does not perform obstacle avoidance, mission state management, arming, mode switching, or failsafe ownership.

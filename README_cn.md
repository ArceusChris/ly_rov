# LY ROV 视觉管线

这是一个面向 ROV 的 ROS 2 视觉管线：

1. 使用 V4L2 采集相机图像；
2. 使用 UDCP 算法去雾；
3. 在去雾后的图像上运行 YOLOv8 分割；
4. 使用分割 mask 完成水下管道循迹；
5. 通过 MAVLink UDP 向指定 IP 的从机发送运动指令。

默认假设相机垂直朝下安装。

## 目录结构

```text
ros2_ws/src/camera_driver        V4L2 相机组件，发布 bgr8 图像
ros2_ws/src/dehaze               UDCP 去雾组件，以及相机+去雾 launch
ros2_ws/src/dehaze_segmentation  YOLOv8-seg 配置和 launch 包
ros2_ws/src/rov_pipe_tracker     管道循迹节点和全链路 launch
hobot_dnn                        D-Robotics DNN 示例依赖
mavlink                          用于生成 C 头文件的 MAVLink submodule
ardupilot                        ArduPilot 源码/submodule
```

## Submodule

构建前先初始化 submodule：

```bash
git submodule update --init --recursive
```

`rov_pipe_tracker` 会在 CMake 构建时从 `mavlink` submodule 生成 MAVLink C 头文件，所以 `mavlink/pymavlink` 这个嵌套 submodule 也需要初始化。

## 构建

```bash
cd ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

YOLOv8 分割节点复用 `dnn_node_example`。目标机器上需要有 D-Robotics 运行环境、`ai_msgs`、`dnn_node`、模型文件，以及 OpenCV/cv_bridge 等依赖。

默认 YOLOv8-seg 模型路径：

```text
/opt/hobot/model/x5/basic/yolov8_seg_640x640_nv12.bin
```

如果平台或模型路径不同，修改：

```text
ros2_ws/src/dehaze_segmentation/config/yolov8segworkconfig.json
```

## 运行

只运行相机和去雾：

```bash
ros2 launch dehaze camera_dehaze_container.launch.py \
  device:=/dev/video0 width:=320 height:=240 fps:=30
```

运行相机、去雾和 YOLOv8 分割：

```bash
ros2 launch dehaze_segmentation dehaze_yolov8seg_container.launch.py \
  device:=/dev/video0 width:=640 height:=480 fps:=30
```

运行完整管道循迹链路：

```bash
ros2 launch rov_pipe_tracker pipe_follow_container.launch.py \
  device:=/dev/video0 \
  target_ip:=192.168.2.2 \
  target_port:=14550
```

## 开机自启：单机控制默认

当前开机自启默认使用单机 OpenCV 管道循迹：

```text
pipe_follow_cv_container.launch.py
```

安装或更新自启服务：

```bash
cd /home/sunrise/ly_rov
./scripts/install_pipe_follow_autostart.sh
```

日志：

```bash
sudo journalctl -u rov-pipe-follow.service -f
tail -f /home/sunrise/ly_rov/log/autostart/ardusub.log
tail -f /home/sunrise/ly_rov/log/autostart/pipe_follow_cv.log
```

## 8080 Web UI 远端处理窗口

单机控制链保持不变，10 号设备继续运行 `pipe_follow_cv_container.launch.py`。额外显示链路为：

```text
10 号设备 8080 Web UI /raw.mjpg
  -> HTTP MJPEG
  -> 11 号设备算法处理
  -> 11 号设备 HTTP MJPEG /processed.mjpg
  -> 10 号设备 8080 Web UI 的 Remote Processing 窗口
```

这种方式不使用 ROS 2 topic 传输图像，适合提高跨设备图像传输速率。

10 号设备启动 Web UI 时需要把第三窗口指向 11 号设备的处理流：

```bash
cd /home/sunrise/ly_rov
ROS_LAUNCH_ARGS='processed_stream_url:=http://192.168.127.11:8090/processed.mjpg' \
./scripts/install_pipe_follow_autostart.sh
sudo systemctl restart rov-pipe-follow.service
```

11 号设备启动远端处理预览，不会下发控制命令：

```bash
cd /home/sunrise/ly_rov
./scripts/remote_mjpeg_processing_preview.py \
  --input http://192.168.127.10:8080/raw.mjpg \
  --host 0.0.0.0 \
  --port 8090 \
  --path /processed.mjpg
```

检查：

```bash
curl --max-time 2 -o /dev/null -v http://192.168.127.10:8080/raw.mjpg
curl --max-time 2 -o /dev/null -v http://192.168.127.11:8090/processed.mjpg
```

## 11 号设备算法接入接口

当前双设备方案已经改为 HTTP/MJPEG 接口，替代原来的 `ros2_ws` ROS 2 图像 topic 接口。

11 号设备算法只需要使用这两个数据接口：

```text
输入:
  http://192.168.127.10:8080/raw.mjpg

输出:
  http://192.168.127.11:8090/processed.mjpg
```

不要再用这些旧 ROS 2 图像传输命令：

```bash
ros2 topic hz /image_raw
ros2 launch rov_pipe_tracker split_processing_cv.launch.py
ros2 launch rov_pipe_tracker split_processing_yolov8.launch.py
ros2 launch rov_pipe_tracker remote_processing_preview_cv.launch.py
```

11 号设备算法接入说明见：[docs/device_11_data_interfaces_cn.md](docs/device_11_data_interfaces_cn.md)。

如果要把分割算法也放到 11 号设备运行，并让 10 号设备继续使用远程 mask 做循迹控制，见：
[docs/remote_segmentation_deployment_cn.md](docs/remote_segmentation_deployment_cn.md)。

OpenCV 管道分割、视频 Web UI 和手动控制 Web UI：

```bash
ros2 launch rov_pipe_tracker pipe_follow_cv_container.launch.py \
  device:=/dev/video0 \
  target_port:=14550
```

默认端口：

```text
8080  视频、管道 mask 叠加监看、自动控制指令日志
8081  手动运动控制
```

如果分割节点已经单独运行，只启动循迹节点：

```bash
ros2 launch rov_pipe_tracker pipe_tracker.launch.py \
  mask_topic:=hobot_dnn_segmentation \
  target_ip:=192.168.2.2 \
  target_port:=14550
```

## Topic

以下 topic 是 10 号设备单机控制链内部使用的 ROS 2 topic。当前 11 号设备算法接入不使用这些 topic，而是使用 HTTP/MJPEG 接口。

```text
image_raw               camera_driver 发布的原始 BGR 图像
image_dehazed           dehaze 发布的去雾后 BGR 图像
hobot_dnn_segmentation  ai_msgs/msg/PerceptionTargets 分割结果
mavlink_manual_control_command
                        旧分布式控制方案使用的控制命令，[x, y, z, r]
```

完整链路 launch 会把相机、去雾、分割、循迹节点放进同一个 ROS 2 component container，并开启 `use_intra_process_comms`。

## 管道循迹参数

常用参数：

```text
target_ip           MAVLink UDP 目标 IP
target_port         MAVLink UDP 目标端口，默认 14550
mavlink_enabled     是否由 pipe_tracker 直接发送 MAVLink；分布式算法端设为 false
output_command_topic
                    旧分布式控制方案下 pipe_tracker 发布控制命令的 topic
mask_topic          分割结果 topic
mask_label          要跟踪的 label；-1 表示所有非零 mask 像素
min_pixels          判定为有效管道 mask 的最少像素数
desired_angle_deg   图像坐标系里的期望管道方向，默认 90
forward_axis        MAVLink MANUAL_CONTROL 的 x 轴，范围 -1000..1000
z_axis              MAVLink MANUAL_CONTROL 的 z 轴，范围 0..1000，默认 500
lateral_gain        图像横向偏差到 MANUAL_CONTROL y 轴的增益
yaw_gain            管道角度偏差到 MANUAL_CONTROL r 轴的增益
lateral_sign        横移方向反号，不用改代码
yaw_sign            偏航方向反号，不用改代码
stale_timeout_s     mask 超时后发送中立指令
```

对于垂直朝下的相机，建议先用较小的 `forward_axis`，在推进器禁用或受限的状态下测试，再根据实艇效果调整 `lateral_sign`、`yaw_sign` 和 `desired_angle_deg`。

## MAVLink 输出

`rov_pipe_tracker` 会发送：

```text
HEARTBEAT       每秒一次
MANUAL_CONTROL  按 command_rate_hz 发送
```

当没有有效 mask 或 mask 超时时，发送中立控制：

```text
x=0 y=0 z=500 r=0
```

## 手动控制 Web UI

手动控制 UI 默认地址：

```text
http://<ROV_IP>:8081
```

它发布 `manual_control_command`，消息格式为：

```text
[x, y, z, r, override_enabled]
```

其中 `x/y/r` 范围为 `-1000..1000`，`z` 范围为 `0..1000`，`z=500` 表示中立。手动覆盖启用时，手动命令优先于自动循迹；按 `RELEASE AUTO` 后才释放回自动循迹。

## 安全说明

首次测试时请固定 ROV，或禁用/限制推进器，先确认 MAVLink 通信和控制方向。当前循迹节点故意保持简单：它只跟踪最大的有效分割 mask，不负责避障、任务状态管理、解锁、模式切换或整机 failsafe。

# 远程分割部署接口

本文档说明如何把管道分割算法放到 11 号设备运行，同时让 10 号设备继续执行 Web UI、循迹和 MAVLink 控制。

## 数据流

```text
10 号设备相机
  |
  | HTTP MJPEG
  | http://192.168.127.10:8080/raw.mjpg
  v
11 号设备远程分割服务
  |
  | HTTP MJPEG mask
  | http://192.168.127.11:8090/mask.mjpg
  v
10 号设备 remote_mask_mjpeg_bridge
  |
  | ROS 2 ai_msgs/msg/PerceptionTargets
  | remote_pipe_segmentation
  v
10 号设备 pipe_tracker -> MAVLink 控制
```

11 号设备还会提供可视化叠加流：

```text
http://192.168.127.11:8090/processed.mjpg
```

10 号设备 Web UI 也直接使用 11 号设备提供的远程显示流：

```text
Raw Camera         10 号设备 /raw.mjpg
Pipe Overlay       11 号设备 /processed.mjpg
Remote Processing  11 号设备 /mask.mjpg
```

其中 `Pipe Overlay` 是远端分割后的叠加画面，`Remote Processing` 是远端二值 mask 画面。

## HTTP 接口

### 10 号设备提供给 11 号设备

```text
URL:      http://192.168.127.10:8080/raw.mjpg
协议:     HTTP multipart MJPEG
内容:     原始相机 JPEG 帧
消费者:   11 号设备远程分割服务
```

### 11 号设备提供给 10 号设备

```text
URL:      http://192.168.127.11:8090/mask.mjpg
协议:     HTTP multipart MJPEG
内容:     单通道二值 mask，白色为管道，黑色为背景
消费者:   10 号设备 remote_mask_mjpeg_bridge
```

```text
URL:      http://192.168.127.11:8090/processed.mjpg
协议:     HTTP multipart MJPEG
内容:     原图 + mask 叠加调试画面
消费者:   10 号设备 8080 Web UI 的 Remote Processing 窗口
```

```text
URL:      http://192.168.127.11:8090/metadata.json
协议:     HTTP JSON
内容:     最近一帧的分割统计信息
用途:     调参和排查
```

## 11 号设备启动远程分割服务

```bash
cd /home/sunrise/ly_rov
./ros2_ws/src/rov_pipe_tracker/scripts/remote_cv_pipe_segmenter_server.py \
  --input http://192.168.127.10:8080/raw.mjpg \
  --host 0.0.0.0 \
  --port 8090 \
  --max-fps 15 \
  --jpeg-quality 80 \
  --mask-jpeg-quality 95
```

常用分割参数：

```bash
--min-red 150
--min-green 150
--min-blue 150
--max-channel-diff 45
--max-red-blue-diff 55
--min-area 3000
--min-height-ratio 0.18
--min-aspect 0.5
--max-aspect 2.0
--erode-kernel-size 1
```

这些参数和本机 OpenCV 分割节点的含义一致。

## 10 号设备启动远程分割控制链

先重新编译工作区：

```bash
cd /home/sunrise/ly_rov/ros2_ws
colcon build --packages-select rov_pipe_tracker
source install/setup.bash
```

手动启动：

```bash
ros2 launch rov_pipe_tracker pipe_follow_remote_segmentation.launch.py \
  mask_stream_url:=http://192.168.127.11:8090/mask.mjpg \
  overlay_stream_url:=http://192.168.127.11:8090/processed.mjpg \
  processed_stream_url:=http://192.168.127.11:8090/mask.mjpg
```

自启部署：

```bash
cd /home/sunrise/ly_rov
ROS_LAUNCH_FILE=pipe_follow_remote_segmentation.launch.py \
ROS_AUTOSTART_LOG=pipe_follow_remote_segmentation.log \
ROS_LAUNCH_ARGS='mask_stream_url:=http://192.168.127.11:8090/mask.mjpg overlay_stream_url:=http://192.168.127.11:8090/processed.mjpg processed_stream_url:=http://192.168.127.11:8090/mask.mjpg' \
./scripts/install_pipe_follow_autostart.sh
sudo systemctl restart rov-pipe-follow.service
```

## ROS 2 内部接口

10 号设备上的 `remote_mask_mjpeg_bridge` 会把远程 mask 转成：

```text
topic: remote_pipe_segmentation
type:  ai_msgs/msg/PerceptionTargets
```

消息内容：

```text
target.type: "pipe"
capture.img.width:  mask 宽度
capture.img.height: mask 高度
capture.img.step:   1
capture.features:   展平后的 float mask，0.0 为背景，1.0 为管道
```

`pipe_tracker` 订阅 `remote_pipe_segmentation` 后按原来的逻辑计算 mask 质心、主方向和控制指令。

## 调试命令

在 11 号设备检查能否拉到 10 号原始图：

```bash
curl --max-time 2 -o /dev/null -v http://192.168.127.10:8080/raw.mjpg
```

在 10 号设备检查能否拉到 11 号分割结果：

```bash
curl --max-time 2 -o /dev/null -v http://192.168.127.11:8090/mask.mjpg
curl --max-time 2 -o /dev/null -v http://192.168.127.11:8090/processed.mjpg
curl --max-time 2 http://192.168.127.11:8090/metadata.json
```

检查 ROS mask 是否发布：

```bash
ros2 topic hz /remote_pipe_segmentation
ros2 topic echo /remote_pipe_segmentation --once
```

打开 10 号设备 Web UI：

```text
http://192.168.127.10:8080
```

## 故障排查

如果 11 号提示无法打开 `raw.mjpg`，先确认 10 号 Web UI 已启动，并且两台设备网络互通。

如果 10 号 `remote_pipe_segmentation` 没有频率，检查 11 号 `mask.mjpg` 是否能用 `curl` 打开。

如果控制一直保持中立，检查 `metadata.json` 里的 `output_pixels` 是否长期为 0；如果是，需要调整阈值或确认画面里管道颜色是否符合当前白色管道分割条件。

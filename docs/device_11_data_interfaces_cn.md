# 11 号设备 HTTP/MJPEG 算法接入接口

本文档说明如何在 `192.168.127.11` 接入你的图像处理算法。该方案不使用 ROS 2 topic 传输图像，而是使用和 Web UI 类似的 HTTP MJPEG 流。

这个 HTTP/MJPEG 接口就是当前 11 号设备算法接入接口，用来替代之前 `ros2_ws` 里的 ROS 2 图像 topic 接口。11 号设备算法不需要订阅 `/image_raw`，也不需要发布 `/remote_processed_image`。

当前目标：

```text
10 号设备：
  保持原本单机检测控制链不变
  运行 8080 Web UI
  提供原始图像 MJPEG: http://192.168.127.10:8080/raw.mjpg
  Web UI 第三个窗口显示 11 号设备处理结果

11 号设备：
  拉取 10 号设备 raw.mjpg
  运行你的图像处理算法
  对外提供 processed.mjpg
  不发布运动控制命令
  不影响 10 号设备现有检测控制链
```

## 数据流

```text
10 号设备 8080 Web UI
  |
  | HTTP MJPEG
  | http://192.168.127.10:8080/raw.mjpg
  v
11 号设备你的算法
  |
  | HTTP MJPEG
  | http://192.168.127.11:8090/processed.mjpg
  v
10 号设备 8080 Web UI
  |
  v
Remote Processing 新窗口
```

现有控制链仍然只在 10 号设备内部运行：

```text
10 号设备相机 -> 本机 OpenCV 检测 -> pipe_tracker -> MAVLink 控制
```

11 号设备的处理结果只用于显示。

## 10 号设备接口

### 输入给 11 号设备的原始图像流

```text
URL:  http://192.168.127.10:8080/raw.mjpg
协议: HTTP multipart MJPEG
用途: 11 号设备算法输入
```

检查：

```bash
curl --max-time 2 -o /dev/null -v http://192.168.127.10:8080/raw.mjpg
```

浏览器也可以直接打开：

```text
http://192.168.127.10:8080/raw.mjpg
```

### 8080 Web UI 第三窗口配置

10 号设备 Web UI 的第三个窗口 `Remote Processing` 可以直接显示 11 号设备的处理结果流。

配置自启服务：

```bash
cd /home/sunrise/ly_rov
ROS_LAUNCH_ARGS='processed_stream_url:=http://192.168.127.11:8090/processed.mjpg' \
./scripts/install_pipe_follow_autostart.sh
sudo systemctl restart rov-pipe-follow.service
```

打开：

```text
http://192.168.127.10:8080
```

第三个窗口会加载：

```text
http://192.168.127.11:8090/processed.mjpg
```

## 11 号设备接口

### 你的算法输入

```text
Input URL:
  http://192.168.127.10:8080/raw.mjpg

Format:
  MJPEG/JPEG frames

OpenCV 读取方式:
  cv2.VideoCapture("http://192.168.127.10:8080/raw.mjpg")
```

### 你的算法输出

```text
Output URL:
  http://192.168.127.11:8090/processed.mjpg

Format:
  HTTP multipart MJPEG

用途:
  10 号设备 8080 Web UI 的 Remote Processing 窗口
```

推荐：

```text
输出 JPEG 质量: 70-85
输出帧率: 10-20 FPS
无线网络优先: 320x240 或 640x480
```

## 使用仓库内置 HTTP 预览脚本

11 号设备运行：

```bash
cd /home/sunrise/ly_rov
./scripts/remote_mjpeg_processing_preview.py \
  --input http://192.168.127.10:8080/raw.mjpg \
  --host 0.0.0.0 \
  --port 8090 \
  --path /processed.mjpg \
  --max-fps 15 \
  --jpeg-quality 80
```

该脚本做的是：

```text
拉取 10 号 raw.mjpg
执行 process_frame(frame)
输出 11 号 processed.mjpg
```

你可以直接修改脚本里的这个函数接入自己的算法：

```python
def process_frame(frame):
    # Replace this block with the algorithm that should be shown in the 8080 UI.
    result = frame.copy()
    return result
```

## 自定义算法最小模板

你的算法只需要实现这两个动作：

```text
读取:
  http://192.168.127.10:8080/raw.mjpg

提供:
  http://192.168.127.11:8090/processed.mjpg
```

最小 Python 结构：

```python
import cv2


input_url = "http://192.168.127.10:8080/raw.mjpg"
cap = cv2.VideoCapture(input_url)

while True:
    ok, frame = cap.read()
    if not ok:
        break

    # 在这里写你的算法。
    result = frame

    # result 需要通过 HTTP MJPEG server 输出到：
    # http://192.168.127.11:8090/processed.mjpg
```

如果不想自己写 HTTP server，直接基于：

```text
scripts/remote_mjpeg_processing_preview.py
```

改 `process_frame(frame)` 即可。

## 不要使用的接口

为了保持当前检测控制链不变，11 号设备算法不要发布或调用这些控制接口：

```text
/mavlink_manual_control_command
/manual_control_command
/pipe_cv_segmentation
/hobot_dnn_segmentation
/pipe_tracker/*
/mavlink_manual_control_bridge/*
```

当前 11 号设备算法只负责：

```text
HTTP 输入 raw.mjpg
HTTP 输出 processed.mjpg
```

## 调试命令

在 11 号设备检查能否拉到 10 号原始图：

```bash
curl --max-time 2 -o /dev/null -v http://192.168.127.10:8080/raw.mjpg
```

在 10 号设备检查能否看到 11 号处理结果：

```bash
curl --max-time 2 -o /dev/null -v http://192.168.127.11:8090/processed.mjpg
```

如果 curl 正常，但浏览器第三窗口不显示，重启 10 号服务：

```bash
sudo systemctl restart rov-pipe-follow.service
```

查看 10 号日志：

```bash
tail -f /home/sunrise/ly_rov/log/autostart/pipe_follow_cv.log
```

## 常见问题

### 11 号设备打不开 raw.mjpg

检查：

```bash
ping 192.168.127.10
curl --max-time 2 -o /dev/null -v http://192.168.127.10:8080/raw.mjpg
sudo systemctl status rov-pipe-follow.service
```

### 10 号 Web UI 第三窗口打不开 processed.mjpg

检查 11 号脚本是否运行：

```bash
curl --max-time 2 -o /dev/null -v http://192.168.127.11:8090/processed.mjpg
```

检查 10 号 Web UI 是否配置了 `processed_stream_url`：

```bash
sudo systemctl cat rov-pipe-follow.service | grep processed_stream_url
```

### 帧率仍然低

优先调低 JPEG 质量和输出帧率：

```bash
./scripts/remote_mjpeg_processing_preview.py \
  --input http://192.168.127.10:8080/raw.mjpg \
  --jpeg-quality 70 \
  --max-fps 10
```

如果 10 号 `/raw.mjpg` 本身帧率低，需要优化 10 号相机采集和本机处理链。

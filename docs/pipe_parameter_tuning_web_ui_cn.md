# Pipe Parameter Tuning 调参网页说明

本文档说明 `Pipe Parameter Tuning` 网页中每个参数的含义、默认值、范围和调参方向。

该网页由 `parameter_tuning_web_ui` 节点提供，默认地址：

```text
http://<设备IP>:8082
```

在当前单机 OpenCV 管道循迹自启链路中，默认是：

```text
pipe_follow_cv_container.launch.py
```

网页会实时读写两个 ROS 2 节点的参数：

```text
Segmentation -> /cv_pipe_segmenter
Control      -> /pipe_tracker
```

## 页面按钮

| 按钮 | 含义 |
| --- | --- |
| `Refresh` | 立即从 ROS 参数服务读取当前参数值，并刷新网页显示。页面也会每 3 秒自动刷新一次。 |
| `Launch Defaults` | 把网页内置的启动默认值重新写回参数服务。注意这是运行时写入，不等于永久修改 launch 文件。 |

## 调参是否会永久保存

网页调参是运行时修改 ROS 参数：

```text
网页 -> /api/set -> ROS SetParameters -> 节点内存参数
```

服务重启后，参数会回到 launch 文件或自启脚本传入的默认值。若要永久保存，需要把确认后的值写入启动参数，例如：

```bash
cd /home/sunrise/ly_rov
ROS_LAUNCH_ARGS='min_red:=150 min_green:=150 min_blue:=150 max_channel_diff:=45 min_area:=3000 forward_axis:=150' \
./scripts/install_pipe_follow_autostart.sh
sudo systemctl restart rov-pipe-follow.service
```

## 分割参数 Segmentation

这些参数控制 OpenCV 如何从相机图像中提取“管道掩膜”。处理逻辑大致是：

```text
读取 RGB/BGR 三通道
亮度筛选: R > min_red, G > min_green, B > min_blue
白/灰色筛选: max(R,G,B) - min(R,G,B) <= max_channel_diff
木板偏黄棕排除: R - B <= max_red_blue_diff
形态学闭运算/开运算
连通域过滤
选择最佳连通域
可选腐蚀 erode
```

| 参数 | 默认值 | 范围 | 含义 | 调大效果 | 调小效果 |
| --- | ---: | ---: | --- | --- | --- |
| `min_red` | `150` | `0-255` | 红色通道下限。管道像素需要满足 `R > min_red`。 | 更严格，只保留更亮的红通道区域。 | 更容易保留暗一些的管道，但背景也可能进来。 |
| `min_green` | `150` | `0-255` | 绿色通道下限。管道像素需要满足 `G > min_green`。 | 更严格，只保留更亮的绿色通道区域。 | 更容易保留暗一些或偏色的管道。 |
| `min_blue` | `150` | `0-255` | 蓝色通道下限。管道像素需要满足 `B > min_blue`。 | 更严格，并能明显压掉黄棕木板。 | 更容易保留偏黄或水下偏色后的白管。 |
| `max_channel_diff` | `45` | `0-255` | 三通道最大差值。`max(R,G,B)-min(R,G,B)` 小于该值才认为接近白/灰。 | 更宽松，偏色白管更容易保留，但木板/反光更容易进来。 | 更严格，只保留更纯的白/灰，可能漏检偏色管道。 |
| `max_red_blue_diff` | `55` | `-255-255` | 黄棕木板排除阈值。木板通常 `R-B` 较大，超过该值会被排除。 | 更宽松，偏黄管道更容易保留，但木板可能进来。 | 更严格，更强地排除黄棕背景，可能漏检暖色光下的管道。 |
| `min_area` | `3000` | `0-100000` | 连通域最小面积，单位是像素。小于该面积的候选会被过滤。 | 过滤更多小噪声，但远处/细小管道可能被滤掉。 | 能保留更小目标，但噪声点和碎片会增多。 |
| `min_height_ratio` | `0.18` | `0-1` | 连通域最小高度比例。候选高度必须大于图像高度乘以该比例。 | 要求目标更高，能排掉横向小块噪声。 | 允许更短的目标，远处管道更容易保留。 |
| `min_aspect` | `0.5` | `0-10` | 最小长宽比，代码中使用 `height / width`。 | 更偏向竖长目标，横向/短粗目标会被拒绝。 | 更容易接受宽而矮的区域。 |
| `max_aspect` | `2.0` | `0-20` | 最大长宽比，代码中使用 `height / width`。大于该值会被拒绝；设为 `0` 时相当于关闭上限过滤。 | 允许更细长的目标。 | 过滤过细长区域，避免水纹、边缘光带误检。 |
| `erode_kernel_size` | `1` | `1-31` | 最终选中掩膜的腐蚀核大小。`1` 表示基本不腐蚀。 | 掩膜会变细、边缘收缩，能去掉毛刺，但面积变小。 | 保留更多原始掩膜面积。 |

### 分割调参建议

若管道漏检：

```text
优先降低 min_red / min_green / min_blue
再降低 min_area
必要时增大 max_channel_diff
暖色光下可以适当增大 max_red_blue_diff
```

若背景误检很多：

```text
优先增大 min_blue
再减小 max_channel_diff
若木板进入较多，减小 max_red_blue_diff
增大 min_area
根据误检形状调整 min_aspect / max_aspect
```

若掩膜边缘太毛或太粗：

```text
适当增大 erode_kernel_size
```

但 `erode_kernel_size` 太大会让管道面积变小，可能导致后面的 `min_pixels` 判定失败。

## 控制参数 Control

这些参数控制 `/pipe_tracker` 如何把管道掩膜转换为 MAVLink `MANUAL_CONTROL` 指令。

控制输出四个轴：

```text
x = forward_axis
y = lateral_sign * lateral_gain * x_error
z = z_axis
r = yaw_sign * yaw_gain * yaw_error
```

其中：

```text
x_error   = 管道中心相对画面中心的水平偏差，归一化到约 -1 到 1
yaw_error = 管道角度与 desired_angle_deg 的偏差，归一化到约 -1 到 1
```

最终 `x/y/r` 会限制在 `-1000` 到 `1000`，`z` 会限制在 `0` 到 `1000`。

| 参数 | 默认值 | 范围 | 含义 | 调大效果 | 调小效果 |
| --- | ---: | ---: | --- | --- | --- |
| `manual_control_enabled` | `true` | `true/false` | 是否允许 `/pipe_tracker` 持续发送控制流。 | `true` 时自动/手动覆盖控制可以输出。 | `false` 时停止控制流，并发送中位安全值 `x=0 y=0 z=500 r=0`。 |
| `min_pixels` | `80` | `0-100000` | 有效掩膜最少像素数。少于该值认为没有可靠管道。 | 更严格，减少小噪声触发控制。 | 更灵敏，远处/小目标可触发，但误检风险增加。 |
| `desired_angle_deg` | `90` | `-180-180` | 希望管道在画面中的目标角度，单位度。系统会根据实际角度和该角度的差值产生 yaw 控制。 | 目标角度按数值变化；常见竖直管道用 `90`。 | 同左，取决于相机安装方向和管道在画面中的期望姿态。 |
| `forward_axis` | `150` | `-1000-1000` | MAVLink `MANUAL_CONTROL.x`，通常表示前进/后退速度指令。 | 前进更快；数值太大会来不及横向/航向修正。 | 前进更慢；便于观察和调参。负值通常表示后退。 |
| `z_axis` | `500` | `0-1000` | MAVLink `MANUAL_CONTROL.z`，油门/升沉轴中位通常是 `500`。 | 可能上浮或增大垂向推力，具体方向取决于飞控映射。 | 可能下潜或减小垂向推力，具体方向取决于飞控映射。 |
| `lateral_gain` | `450` | `0-1000` | 水平偏差到横移指令 `y` 的比例系数。 | 横向修正更猛，跟线更积极，但可能左右震荡。 | 横向修正更柔，稳定但可能跟不上偏差。 |
| `yaw_gain` | `450` | `0-1000` | 角度偏差到旋转指令 `r` 的比例系数。 | 转向修正更猛，角度收敛快，但可能抖动。 | 转向更柔，稳定但角度修正慢。 |
| `lateral_sign` | `1` | `-1 或 1` | 横移方向符号。用于修正相机坐标与飞控横移方向是否相反。 | `1` 表示按当前计算方向输出。 | `-1` 表示反向输出。若看到越偏越远，应切换符号。 |
| `yaw_sign` | `1` | `-1 或 1` | 偏航方向符号。用于修正图像角度误差与飞控旋转方向是否相反。 | `1` 表示按当前计算方向输出。 | `-1` 表示反向输出。若看到越转越歪，应切换符号。 |
| `command_rate_hz` | `10` | `1-30` | 控制指令发送频率，单位 Hz。修改后会重建发送定时器。 | 控制刷新更快，但占用更多通信和处理资源。 | 控制刷新更慢，动作更迟缓但更轻量。 |
| `stale_timeout_s` | `0.5` | `0-5` | 掩膜观测超时时间。超过该时间没有新有效观测，就输出中位安全值。 | 容忍短暂掉帧，但丢目标后继续执行旧指令的时间更长。 | 丢帧后更快停住，但可能因偶发掉帧频繁中断。 |
| `manual_override_timeout_s` | `0.5` | `0-5` | 手动控制覆盖超时时间。网页/手柄手动命令超过该时间未更新，就回中位安全值。 | 手动命令断流后维持更久。 | 断流后更快回安全值。 |

### 控制调参建议

第一次下水或不确定方向时：

```text
forward_axis 先设小一些，例如 50-100
lateral_gain / yaw_gain 先保守，例如 200-350
确认 lateral_sign 和 yaw_sign 方向正确
```

若机器人看到管道偏右却横移方向更偏：

```text
把 lateral_sign 从 1 改成 -1，或从 -1 改成 1
```

若机器人角度越修越歪：

```text
把 yaw_sign 从 1 改成 -1，或从 -1 改成 1
```

若左右来回摆动：

```text
降低 lateral_gain
降低 yaw_gain
降低 forward_axis
```

若跟踪太慢：

```text
适当增大 lateral_gain
适当增大 yaw_gain
在稳定后再增大 forward_axis
```

## 常用查看命令

查看调参网页节点日志：

```bash
sudo journalctl -u rov-pipe-follow.service -f
```

查看 OpenCV 分割统计：

```bash
tail -f /home/sunrise/ly_rov/log/autostart/pipe_follow_cv.log
```

分割节点日志中会周期性打印：

```text
threshold_px
morph_px
components
rejected_area
rejected_min_aspect
rejected_max_aspect
largest
best_label
selected_px
output_px
params(...)
```

这些字段可用于判断是阈值阶段没选中，还是连通域过滤阶段被过滤。

控制节点日志中会周期性打印：

```text
pipe observation
manual_control
pipe_tracker_command_log
```

重点看：

```text
valid       是否识别到有效管道
pixels      有效掩膜像素数
center      管道中心点
angle_deg   管道主方向角度
x_error     横向归一化误差
yaw_error   角度归一化误差
x/y/z/r     实际输出控制轴
```

## 推荐调参顺序

1. 先只调 `Segmentation`，让掩膜稳定覆盖管道，少覆盖背景。
2. 再调 `min_pixels`，保证小噪声不会触发控制，真实管道不会被判无效。
3. 低速设置 `forward_axis`，确认 `lateral_sign` 和 `yaw_sign` 方向正确。
4. 调 `lateral_gain` 和 `yaw_gain`，让修正足够快但不振荡。
5. 最后再逐步提高 `forward_axis`。

不要一开始同时大幅修改多个参数。每次只改一到两个参数，更容易判断变化来源。

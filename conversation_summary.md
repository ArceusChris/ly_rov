# 对话整理

## 目标
梳理一个新的 ArduPilot 仓库如何配成可被 ROS2 节点控制的 ROV 飞控系统。

## 核心结论
- 飞控本体仍然是 `ArduSub`
- ROS2 节点负责视觉、决策和控制量映射
- 最终控制出口是 MAVLink `MANUAL_CONTROL`
- 单个 ROS2 节点可以做完整闭环，但不能替代飞控本身

## 项目结构理解
当前仓库主线是 ROS2 视觉管线：
- `camera_driver`：V4L2 采集，发布 `image_raw`
- `dehaze`：UDCP 去雾，发布 `image_dehazed`
- `dehaze_segmentation`：YOLOv8-seg launch/config 包
- `rov_pipe_tracker`：管道循迹、Web UI、MAVLink 输出

参考文件：
- [README.md](README.md)
- [structure.md](structure.md)

## ArduPilot 侧需要改的地方
如果你要把一台新的 Linux 设备做成 ArduSub 飞控，重点是这几类文件：

### 1. 板级定义
[ardupilot/libraries/AP_HAL_Linux/hwdef/subrov/hwdef.dat](ardupilot/libraries/AP_HAL_Linux/hwdef/subrov/hwdef.dat)

作用：
- 定义硬件能力
- 定义推进器/串口/传感器约束
- 关闭 GPS、罗盘、避障等不需要的功能

### 2. 默认参数
[ardupilot/libraries/AP_HAL_Linux/boards/subrov/defaults.parm](ardupilot/libraries/AP_HAL_Linux/boards/subrov/defaults.parm)

作用：
- 定义机体默认飞行配置
- `FRAME_CONFIG`
- `SERVO1..8_FUNCTION`
- `SERVO_MIN/TRIM/MAX`
- `SERIAL1_PROTOCOL`
- `EAHRS_*`

### 3. 飞控程序入口
[ardupilot/ArduSub/wscript](ardupilot/ArduSub/wscript)

作用：
- 生成 `ardusub`
- 产物通常是 `build/subrov/bin/ardusub`

### 4. 构建命令
```bash
cd ardupilot
git submodule update --init --recursive
./waf configure --board subrov
./waf sub
```

## 飞控通信方式
这个项目用的是 MAVLink。

### ROS2 节点发送的内容
在 [pipe_tracker_node.cpp](ros2_ws/src/rov_pipe_tracker/src/pipe_tracker_node.cpp) 中，节点会发送：
- `HEARTBEAT`
- `MAV_CMD_DO_SET_MODE`
- `MAV_CMD_COMPONENT_ARM_DISARM`
- `MANUAL_CONTROL`

### 控制逻辑
自动控制时：
- 订阅视觉结果
- 提取目标质心和主方向
- 算出横向误差和角度误差
- 映射成 `x/y/z/r`
- 发给飞控

对应实现位置：
- [send_command()](ros2_ws/src/rov_pipe_tracker/src/pipe_tracker_node.cpp)
- [observe()](ros2_ws/src/rov_pipe_tracker/src/pipe_tracker_node.cpp)

### 中立值
没有有效视觉结果时，发送：
```text
x=0 y=0 z=500 r=0
```

## ROV 机体描述
可把当前机体描述成：

```text
8-thruster underwater ROV
ArduSub / subrov
x forward, y right, z up, r yaw right
```

更工程化一点的表述：
- `x`：前进/后退
- `y`：左/右平移
- `z`：上/下
- `r`：偏航
- 默认中立：`x=0 y=0 z=500 r=0`

## ROS2 单节点控制 ROV 的最小步骤
1. 克隆并初始化 `ardupilot` 和 `mavlink`
2. 选择或创建 board 配置
3. 修改 `hwdef.dat`
4. 修改 `defaults.parm`
5. 编译 `ardusub`
6. 建立 MAVLink 通信
7. 写 ROS2 控制节点
8. 加超时保护和中立回退
9. 先在仿真或固定推进器环境测试

## 当前对话里确认的文件
- [hwdef.dat](ardupilot/libraries/AP_HAL_Linux/hwdef/subrov/hwdef.dat)
- [defaults.parm](ardupilot/libraries/AP_HAL_Linux/boards/subrov/defaults.parm)
- [sub.parm](ardupilot/Tools/autotest/default_params/sub.parm)
- [sub-6dof.parm](ardupilot/Tools/autotest/default_params/sub-6dof.parm)
- [start_pipe_follow_stack.sh](scripts/start_pipe_follow_stack.sh)
- [pipe_tracker_node.cpp](ros2_ws/src/rov_pipe_tracker/src/pipe_tracker_node.cpp)

## 结论
如果你只是想让一个 ROS2 节点控制整台 ROV，最关键的是：
- 飞控侧把 `subrov` 或你的新 board 配好
- ROS2 侧把视觉误差转成 `MANUAL_CONTROL`
- 先打通 MAVLink，再接视觉闭环

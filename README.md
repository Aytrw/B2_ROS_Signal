# B2 Native RC Interface

<div align="center">

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-blue.svg)](https://docs.ros.org/en/foxy/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Unitree%20B2-orange.svg)](https://www.unitree.com/b2)

**Unitree B2 原厂遥控器信号无损接入 ROS2 桥接器**

</div>

---

## 📋 概述

本项目为 Unitree B2 四足机器人提供原厂遥控器信号的 ROS2 接入方案。通过被动监听 DDS 话题，实现对遥控器信号的**无损、低延迟**捕获，同时**不干扰原厂控制逻辑**。

### 核心特性

- 🎮 **完整信号捕获** - 摇杆 (4轴) + 按键 (16个) + 机器人状态
- 🔒 **零侵入设计** - 仅订阅不发布，不影响原厂控制
- 📊 **极坐标扩展** - 提供摇杆的模长和角度，便于意图识别
- ⚡ **低延迟** - 微秒级处理延迟
- 🧪 **完整测试** - 包含单元测试和调试工具

---

## 🏗️ 项目结构

```
B2_ROS_Signal/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS2 包描述
├── README.md                   # 本文件
│
├── msg/                        # 自定义消息定义
│   ├── NativeRC.msg            # 主消息（聚合所有数据）
│   ├── JoystickState.msg       # 摇杆状态
│   ├── ButtonState.msg         # 按键状态
│   └── RobotState.msg          # 机器人状态
│
├── include/                    # 头文件
│   └── b2_native_rc_interface/
│       ├── types.hpp           # 公共类型定义
│       ├── bridge/
│       │   └── rc_signal_bridge.hpp
│       └── utils/
│           ├── key_parser.hpp
│           └── joystick_processor.hpp
│
├── src/                        # 源代码
│   ├── bridge/
│   │   └── rc_signal_bridge.cpp
│   ├── nodes/
│   │   ├── rc_bridge_node.cpp      # 主节点
│   │   └── rc_test_subscriber.cpp  # 测试节点
│   └── utils/
│       ├── key_parser.cpp
│       └── joystick_processor.cpp
│
├── launch/                     # 启动文件
│   ├── rc_bridge.launch.py
│   └── rc_bridge_with_test.launch.py
│
├── config/                     # 配置文件
│   └── rc_bridge_params.yaml
│
└── test/                       # 单元测试
    └── test_key_parser.cpp
```

---

## 🔧 依赖项

### 系统要求

- Ubuntu 20.04 (B2 机载系统)
- ROS2 Foxy
- C++17 编译器 (GCC 9.4+)

### ROS2 依赖

```bash
sudo apt install ros-foxy-rclcpp ros-foxy-std-msgs ros-foxy-rmw-cyclonedds-cpp
```

### Unitree 依赖

需要安装 [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)：

```bash
# 克隆 unitree_ros2
cd ~/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2.git

# 编译 CycloneDDS (Foxy 需要手动编译 0.10.x 版本)
cd ~/ros2_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x

cd ~/ros2_ws
colcon build --packages-select cyclonedds rmw_cyclonedds_cpp

# 编译 unitree 消息
colcon build --packages-select unitree_go unitree_api
```

---

## 🚀 编译与安装

```bash
# 进入工作空间
cd ~/ros2_ws/src

# 克隆本项目（如果尚未克隆）
git clone https://github.com/your_username/B2_ROS_Signal.git

# 返回工作空间根目录
cd ~/ros2_ws

# 编译
colcon build --packages-select b2_native_rc_interface

# Source 环境
source install/setup.bash
```

---

## 📖 使用方法

### 选项 A：本地测试（无需 B2 硬件）⭐ 推荐先尝试

**使用模拟器进行开发和测试**：

```bash
# 一键启动完整模拟环境（模拟器 + 桥接节点 + 测试订阅者）
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py

# 或选择不同的模拟模式：
# 自动模式（周期性变化）
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=auto

# 随机模式（压力测试）
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=random

# 静态模式（调试用）
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=static
```

**在另一个终端验证**：
```bash
# 查看输出话题
ros2 topic echo /b2_native_rc_signal

# 检查发布频率
ros2 topic hz /b2_native_rc_signal

# 可视化节点图
rqt_graph
```

📚 **详细本地测试指南**：[docs/LOCAL_TESTING_GUIDE.md](docs/LOCAL_TESTING_GUIDE.md)

---

### 选项 B：真机部署（需要 B2 硬件）

#### 1. 配置网络

确保开发机与 B2 机器人在同一网段：

```bash
# 配置网卡 IP（假设网卡为 eth0）
sudo ip addr add 192.168.123.15/24 dev eth0
sudo ip link set eth0 up

# 测试连通性
ping 192.168.123.161  # B2 主控 IP
```

#### 2. 配置 CycloneDDS (Foxy 必需)

B2 使用 CycloneDDS 0.10.x，需要正确配置才能与机器人通信：

```bash
# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Source 自编译的 CycloneDDS (如果编译在自定义位置)
source ~/ros2_ws/install/setup.bash

# 设置 RMW 实现为 CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 设置网卡（替换 eth0 为实际网卡名，如 enp3s0）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

> ⚠️ **重要**: Foxy 默认的 CycloneDDS 版本与 B2 不兼容，必须手动编译 0.10.x 版本！

#### 3. 启动桥接节点（真机模式）

```bash
# 方式1: 仅启动桥接节点
ros2 launch b2_native_rc_interface rc_bridge.launch.py

# 方式2: 同时启动测试订阅者（推荐调试时使用）
ros2 launch b2_native_rc_interface rc_bridge_with_test.launch.py
```

#### 4. 查看数据

```bash
# 查看话题列表
ros2 topic list

# 查看消息内容
ros2 topic echo /b2_native_rc_signal

# 查看消息频率
ros2 topic hz /b2_native_rc_signal
```

---

## 📨 消息格式

### NativeRC.msg（主消息）

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | std_msgs/Header | 时间戳和帧ID |
| `left_stick` | JoystickState | 左摇杆状态 |
| `right_stick` | JoystickState | 右摇杆状态 |
| `buttons` | ButtonState | 按键状态 |
| `robot_state` | RobotState | 机器人状态 |
| `seq` | uint32 | 消息序列号 |
| `bridge_latency_us` | uint32 | 处理延迟(微秒) |

### JoystickState.msg（摇杆状态）

| 字段 | 范围 | 说明 |
|------|------|------|
| `x` | [-1.0, 1.0] | X轴原始值 |
| `y` | [-1.0, 1.0] | Y轴原始值 |
| `magnitude` | [0.0, 1.414] | 极坐标模长 |
| `angle` | [-π, π] | 极坐标角度(弧度) |

### ButtonState.msg（按键状态）

所有按键均为 `bool` 类型：

| 按键 | 说明 | 按键 | 说明 |
|------|------|------|------|
| `r1` | 右肩键 | `l1` | 左肩键 |
| `r2` | 右扳机 | `l2` | 左扳机 |
| `start` | Start键 | `select` | Select键 |
| `a` | A键 | `b` | B键 |
| `x` | X键 | `y` | Y键 |
| `up` | 方向上 | `down` | 方向下 |
| `left` | 方向左 | `right` | 方向右 |
| `f1` | F1功能键 | `f2` | F2功能键 |

---

## ⚙️ 配置参数

配置文件位于 `config/rc_bridge_params.yaml`：

```yaml
b2_native_rc_bridge:
  ros__parameters:
    # Topic 配置
    rc_input_topic: "/wirelesscontroller"
    output_topic: "/b2_native_rc_signal"
    
    # 超时配置
    rc_timeout_sec: 0.5
    
    # 摇杆死区
    joystick_deadzone: 0.05
    joystick_deadzone_enabled: true
```

---

## 🎯 组合键参考

| 组合键 | 功能 | 检测方式 |
|--------|------|----------|
| L2 + A | 站立锁定/解锁 | `buttons.l2 && buttons.a` |
| L2 + B | 阻尼模式/软急停 | `buttons.l2 && buttons.b` |
| L2 + R2 + X | 特殊动作 | `buttons.l2 && buttons.r2 && buttons.x` |

---

## 🧪 测试

```bash
# 运行单元测试
cd ~/ros2_ws
colcon test --packages-select b2_native_rc_interface

# 查看测试结果
colcon test-result --verbose
```

---

## 📝 开发计划

- [x] Phase 1: 信号接入并发布 rostopic
- [ ] Phase 2: 融合控制权重模型
- [ ] Phase 3: 意图识别与辅助控制

---

## 📄 许可证

本项目采用 BSD-3-Clause 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 🙏 致谢

- [Unitree Robotics](https://www.unitree.com/) - B2 机器人及 SDK
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) - ROS2 消息定义

---

## 📧 联系方式

如有问题或建议，请提交 [Issue](https://github.com/your_username/B2_ROS_Signal/issues)。

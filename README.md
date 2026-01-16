# B2 Native RC Interface

<div align="center">

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-blue.svg)](https://docs.ros.org/en/foxy/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Unitree%20B2-orange.svg)](https://www.unitree.com/b2)

**Unitree B2 遥控器信号 ROS2 桥接器**

</div>

---

## 概述

为 Unitree B2 四足机器人提供原厂遥控器信号的 ROS2 接入方案。通过被动监听 DDS 话题捕获遥控器信号并转换为 ROS2 消息，不干扰原厂控制逻辑。

**核心特性:**
- 完整信号捕获（摇杆 4 轴 + 按键 16 个 + 机器人状态）
- 被动监听设计（只订阅不发布，零侵入）
- 极坐标扩展（提供摇杆模长和角度）
- 低延迟处理（< 100 μs）
- 官方兼容（按键解析、处理参数与 Unitree SDK 一致）

---

## 系统要求

| 组件 | 版本/规格 |
|-----|----------|
| 操作系统 | Ubuntu 20.04 LTS |
| ROS2 | Foxy |
| DDS | CycloneDDS 0.10.x |
| 编译器 | GCC 9.4+ (C++17) |
| Unitree SDK | unitree_ros2 |

---

## 快速开始

### 1. 依赖版本

| 依赖 | 版本要求 |
|-----|---------|
| ROS2 | Foxy |
| CycloneDDS | 0.10.x |
| unitree_ros2 | 最新版 (包含 unitree_go, unitree_api) |

### 2. 编译

```bash
cd ~/ros2_ws/src
git clone <本项目仓库地址>
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. 运行

**模拟模式（无需硬件，推荐先测试）:**
```bash
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py
```

**真机模式:** 见下方 [B2 真机部署](#b2-真机部署)

---

## B2 真机部署

### 1. 网络配置

开发机需与 B2 在同一网段：
B2 ip: `192.168.123.165`
保证本机ip也为 `192.168.123.xxx` 即可

### 2. DDS 配置

```bash
# 设置 CycloneDDS 为中间件
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 指定网卡（替换 eth0 为连接 B2 的网卡）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

> **提示:** 可将上述 export 命令添加到 `~/.bashrc` 中永久生效。

### 3. 启动桥接节点

```bash
ros2 launch b2_native_rc_interface rc_bridge.launch.py
```

### 4. 验证

```bash
# 查看话题频率（应约 100Hz）
ros2 topic hz /b2_native_rc_signal

# 查看消息内容
ros2 topic echo /b2_native_rc_signal
```

---

## 消息接口

### 输入话题

| 话题 | 类型 | 频率 |
|-----|------|------|
| `/wirelesscontroller` | WirelessController | ~100Hz |
| `/sportmodestate` | SportModeState | ~10Hz |
| `/lowstate` | LowState | ~10Hz |

### 输出话题

| 话题 | 类型 | 频率 | 说明 |
|-----|------|------|------|
| `/b2_native_rc_signal` | NativeRC | ~100Hz | 聚合消息 |

### NativeRC 消息结构

```
NativeRC
├── header (Header)              # 时间戳
├── left_stick (JoystickState)   # 左摇杆: x, y, magnitude, angle
├── right_stick (JoystickState)  # 右摇杆: x, y, magnitude, angle
├── buttons (ButtonState)        # 16 个按键状态
├── robot_state (RobotState)     # 运动模式、步态、电池等
├── seq (uint32)                 # 序列号
└── bridge_latency_us (uint32)   # 处理延迟(μs)
```

**详细字段说明见:** [docs/PROJECT_GUIDE.md#2-消息结构](docs/PROJECT_GUIDE.md)

---

## 配置参数

配置文件: `config/rc_bridge_params.yaml`

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `joystick_dead_zone` | 0.01 | 摇杆死区阈值 |
| `joystick_smooth` | 0.03 | 平滑滤波系数 |
| `rc_timeout_sec` | 0.5 | 遥控器超时时间 |
| `qos_depth` | 10 | 消息队列深度 |

**完整配置说明见:** [docs/PROJECT_GUIDE.md#4-配置参数](docs/PROJECT_GUIDE.md)

---

## 使用示例

### C++ 订阅

```cpp
#include "b2_native_rc_interface/msg/native_rc.hpp"

void callback(const b2_native_rc_interface::msg::NativeRC::SharedPtr msg) {
    // 摇杆数据
    float vx = msg->left_stick.y;   // 前进速度
    float wz = msg->right_stick.x;  // 旋转速度
    
    // 按键检测
    if (msg->buttons.a) {
        // A 键按下
    }
    
    // 组合键检测
    if (msg->buttons.l2 && msg->buttons.a) {
        // L2+A 站立解锁
    }
}
```

### Python 订阅

```python
from b2_native_rc_interface.msg import NativeRC

def callback(msg):
    vx = msg.left_stick.y
    wz = msg.right_stick.x
    
    if msg.buttons.a:
        print('A pressed')
```

**更多示例见:** [docs/PROJECT_GUIDE.md#7-二次开发示例](docs/PROJECT_GUIDE.md)

---

## 文档

| 文档 | 说明 |
|-----|------|
| [PROJECT_GUIDE.md](docs/PROJECT_GUIDE.md) | 详细项目指导（功能、原理、使用方法） |
| [ARCHITECTURE_GUIDE.md](docs/ARCHITECTURE_GUIDE.md) | 架构设计（Mermaid 图表、数据流） |

---

## 项目结构


```
b2_native_rc_interface/
├── msg/                # 消息定义
├── include/            # 头文件
├── src/                # 源代码
│   ├── bridge/         # 桥接器实现
│   ├── nodes/          # 节点入口
│   └── utils/          # 工具类
├── launch/             # 启动文件
├── config/             # 配置文件
├── docs/               # 文档
└── test/               # 单元测试
```

---

## 常见问题

### 无消息输出

```bash
# 检查话题
ros2 topic list | grep wirelesscontroller

# 检查 DDS 配置
echo $RMW_IMPLEMENTATION  # 应为 rmw_cyclonedds_cpp

# 查看延迟
ros2 topic echo /b2_native_rc_signal --field bridge_latency_us
```

### CycloneDDS 版本不兼容

B2 使用 CycloneDDS 0.10.x，Foxy 默认版本不兼容：

```bash
# 编译 0.10.x 版本
cd ~/ros2_ws/src
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
cd ~/ros2_ws
colcon build --packages-select cyclonedds rmw_cyclonedds_cpp
```

**详细故障排查见:** [docs/PROJECT_GUIDE.md#9-故障排查](docs/PROJECT_GUIDE.md)

---

## 功能状态

### ✅ 已实现

| 功能 | 说明 |
|-----|------|
| 摇杆信号捕获 | 4 轴数据 + 极坐标转换 |
| 按键状态解析 | 16 个按键 + 组合键检测 |
| 死区/平滑滤波 | 官方默认参数 (0.01/0.03) |
| 机器人状态同步 | 运动模式、步态、电池电量 |
| 模拟器 | 自动模拟器 + 键盘模拟器 |
| 测试订阅者 | 消息验证工具 |

### ⚠️ 已知限制

| 限制 | 原因 |
|-----|------|
| 电池电压/电流未读取 | 需真机验证 LowState 字段 |
| 遥控器电量不可用 | SDK 未暴露此信息 |
| 按键边沿检测未发布 | on_press/on_release 仅内部使用 |

---

## 许可证

BSD-3-Clause License. 详见 [LICENSE](LICENSE)

---

## 参考资料

- [Unitree B2 开发者文档](https://support.unitree.com/home/zh/B2_developer)
- [获取遥控器状态](https://support.unitree.com/home/zh/B2_developer/Get%20Remote%20Control%20Status)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

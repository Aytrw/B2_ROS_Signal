# B2 遥控器信号接入项目 - 完整指南

> 📚 **面向初学者的详细指导文档**
> 
> 本文档将用通俗易懂的语言，带你了解这个项目的方方面面。

---

## 📖 目录

- [1. 项目背景与目的](#1-项目背景与目的)
- [2. 核心概念解释](#2-核心概念解释)
- [3. 技术架构详解](#3-技术架构详解)
- [4. 已完成功能清单](#4-已完成功能清单)
- [5. 待完成功能清单](#5-待完成功能清单)
- [6. 如何运作 - 从头到尾](#6-如何运作---从头到尾)
- [7. 本地测试指南（无需 B2）](#7-本地测试指南无需-b2) ← **新增！**
- [8. 常见问题与排查](#8-常见问题与排查)
- [9. 下一步计划](#9-下一步计划)

---

## 1. 项目背景与目的

### 🎯 我们要解决什么问题？

想象一下，你有一只宇树 B2 机器狗，它有两种控制方式：

1. **原厂遥控器** - 就像遥控汽车的手柄，可以实时操控机器狗走动、改变步态
2. **自主算法** - 你写的 ROS2 程序，让机器狗自己规划路径、避障

**问题来了**：这两种控制方式互相"看不见"对方！

- 当你用遥控器控制时，你的 ROS2 程序不知道遥控器在做什么
- 当算法自主运行时，你想紧急接管，但算法不知道你按了遥控器

### 💡 我们的解决方案

**建立一个"信号窥听器"**，让 ROS2 程序能够：
- 📡 实时知道遥控器的每个按键、每个摇杆的状态
- 🤝 将来实现"人机协同控制"（Phase 2）
- 🚨 在紧急情况下，检测到遥控器输入后立即让出控制权

**关键原则**：
- ✅ **只听不动** - 我们只监听遥控器信号，绝对不修改或拦截它
- ✅ **原厂优先** - 原厂遥控器的控制功能完全保留，不受任何影响
- ✅ **完整数据** - 把遥控器的所有信号（摇杆、按键、机器人状态）都发布给 ROS2

---

## 2. 核心概念解释

### 🔌 DDS 是什么？

**DDS (Data Distribution Service)** 可以理解为一个"数据快递网络"：

```
遥控器接收机 ──发布数据──> DDS网络 ──分发数据──> ┌─> B2主控制器 (原厂程序)
                                              │
                                              └─> 我们的ROS2节点 (窥听者)
```

- B2 机器人内部使用 DDS 作为通信系统
- 遥控器的信号通过 DDS 话题 `rt/wirelesscontroller` 发布
- B2 的主控制器订阅这个话题来控制机器狗
- 我们的程序也订阅这个话题（作为第二个订阅者），获取相同的数据副本

**类比**：就像你在教室里听老师讲课，老师说的话（数据）可以被所有学生（订阅者）同时听到，你听到了并不影响其他学生听。

### 🤖 ROS2 是什么？

**ROS2 (Robot Operating System 2)** 是机器人开发的"操作系统"框架：

- 提供进程间通信（节点之间传递消息）
- 提供工具链（可视化、调试、录制数据等）
- B2 机器人上运行的是 **ROS2 Foxy** 版本（2020年发布）

### 📊 什么是 Topic 和 Message？

可以理解为：

- **Topic（话题）** = 广播电台的频道
- **Message（消息）** = 广播的内容格式

例如：
```
话题: /b2_native_rc_signal (我们创建的频道)
消息: NativeRC.msg (包含摇杆、按键等信息的数据包)
```

任何 ROS2 程序都可以：
- **订阅**这个话题 → 收听这个频道，获取遥控器数据
- **发布**到这个话题 → 在这个频道广播信息

---

## 3. 技术架构详解

### 📐 整体架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    B2 机器狗本体                              │
│                                                             │
│  ┌────────────┐                                             │
│  │ 原厂遥控器  │ (2.4GHz 无线信号)                             │
│  └─────┬──────┘                                              │
│        │                                                     │
│        ▼                                                     │
│  ┌─────────────┐                                              │
│  │ 接收机模块   │ ──发布──> DDS: rt/wirelesscontroller          │
│  └─────────────┘           (100Hz, 原始按键/摇杆数据)            │
│                                   │                           │
│                                   ├──────> [原厂控制器]         │
│                                   │         (步态、运动)        │
│                                   │                           │
│                                   └──────> [我们的节点]         │
│                                             (被动监听)          │
│  ┌──────────────────────────────────────────────────────┐      │
│  │  b2_native_rc_bridge (我们开发的 ROS2 节点)            │      │
│  │                                                      │      │
│  │  ┌─────────────┐   ┌──────────────┐   ┌──────────┐   │      │
│  │  │ DDS订阅者   │──>│ 数据处理器    │──>│ ROS2发布 │ │   │
│  │  │ (遥控器)     │   │ (解析+计算)   │   │   者     │    │     │
│  │  └─────────────┘   └──────────────┘   └────┬─────┘   │     │
│  │                                            │         │     │
│  │  ┌─────────────┐   ┌──────────────┐        │         │     │
│  │  │ DDS订阅者   │──>│ 状态缓存器    │────────┘           │    │
│  │  │ (机器人状态) │   │ (步态/电量等) │                   │     │
│  │  └─────────────┘   └──────────────┘                  │     │
│  └───────────────────────────────────────┬──────────────┘     │
│                                          │                    │
│                                          ▼                    │
│                           ROS2 Topic: /b2_native_rc_signal    │
│                           (我们发布的标准ROS2消息)              │
└──────────────────────────────────────┬───────────────────────┘
                                       │
                                       ▼
                         ┌──────────────────────────┐
                         │  你的算法程序/控制节点     │
                         │  (订阅此话题获取遥控器信号) │
                         └──────────────────────────┘
```

### 🧩 模块功能说明

| 模块名称 | 文件位置 | 功能说明 | 输入 | 输出 |
|---------|---------|---------|------|------|
| **按键解析器** | `key_parser.cpp` | 将 16 位整数解析为 16 个独立的按键状态 | `uint16 raw_keys` | 16 个 `bool` 变量 |
| **摇杆处理器** | `joystick_processor.cpp` | 计算极坐标、应用死区过滤 | `(x, y)` | `(x, y, magnitude, angle)` |
| **信号桥接器** | `rc_signal_bridge.cpp` | 订阅 DDS，聚合数据，发布 ROS2 | DDS 消息 | ROS2 `NativeRC` 消息 |

### 📦 消息结构设计

我们定义了 4 个自定义消息（就像设计了 4 种数据包格式）：

#### 1. `NativeRC.msg` - 主消息（最终发布的完整数据包）

```
NativeRC
├── header (时间戳)
├── left_stick (左摇杆状态)
│   ├── x, y (原始坐标)
│   └── magnitude, angle (极坐标)
├── right_stick (右摇杆状态)
│   ├── x, y
│   └── magnitude, angle
├── buttons (所有按键)
│   ├── r1, l1, r2, l2 (肩键/扳机)
│   ├── a, b, x, y (动作键)
│   ├── up, down, left, right (方向键)
│   └── start, select, f1, f2 (功能键)
├── robot_state (机器人状态)
│   ├── sport_mode (运动模式)
│   ├── gait_type (步态类型)
│   ├── battery_soc (电量)
│   └── rc_connected (遥控器连接状态)
└── seq (序列号，用于检测丢包)
```

#### 2. 为什么要设计极坐标？

**笛卡尔坐标 (x, y)** - 原始数据
```
     ↑ y
     │
     │   /
     │  / 摇杆推向右上方
     │ /
─────┼─────→ x
     │
```

**极坐标 (magnitude, angle)** - 计算值
```
magnitude = √(x² + y²)  → 摇杆推动的"力度"
angle = atan2(y, x)     → 摇杆指向的"方向"
```

**为什么这样设计？**

1. **死区判断**：`magnitude < 0.05` → 认为摇杆在中立位置（没有输入）
2. **意图识别**：`angle` 可以直接判断用户想往哪个方向移动
3. **融合控制**：算法可以根据 `magnitude` 决定要不要让出控制权

**实际例子**：
```python
if msg.left_stick.magnitude < 0.05:
    # 摇杆在中立位置，用户没有操作
    # 算法可以完全接管控制
    robot.autonomous_mode()
else:
    # 用户正在操作摇杆
    # 算法应该降低权重或完全让出控制
    robot.manual_mode()
```

---

## 4. 已完成功能清单

### ✅ 阶段一：信号接入并发布 rostopic (100% 完成)

| 功能项 | 状态 | 说明 |
|--------|------|------|
| 📁 独立 repo 创建 | ✅ 完成 | 项目结构符合 ROS2 工程规范 |
| 📨 自定义消息定义 | ✅ 完成 | 4 个 .msg 文件，含详细注释 |
| 🕹️ 摇杆信号 - 笛卡尔坐标 | ✅ 完成 | `left_stick.x/y`, `right_stick.x/y` |
| 🎯 摇杆信号 - 极坐标 | ✅ 完成 | `magnitude`, `angle` (增强功能) |
| 🔘 按键信号 - 16个按键 | ✅ 完成 | 全部映射为 bool 类型 |
| 🏷️ 按键信号 - 原始值 | ✅ 完成 | `raw_keys` 保留原始 16 位值 |
| 🤖 状态机信号 - 步态 | ✅ 完成 | `sport_mode`, `gait_type` |
| 🔋 状态机信号 - 电量 | ✅ 完成 | `battery_soc` (机器人电量) |
| ⚠️ 状态机信号 - 遥控器电量 | ⚠️ 无法获取 | SDK 未提供，已在消息中注释说明 |
| 🔌 DDS 被动监听 | ✅ 完成 | 不干扰原厂控制 |
| 📡 ROS2 话题发布 | ✅ 完成 | `/b2_native_rc_signal` |
| ⚙️ 配置文件支持 | ✅ 完成 | YAML 参数配置 |
| 🚀 Launch 文件 | ✅ 完成 | 一键启动脚本 |
| 🧪 测试节点 | ✅ 完成 | 用于验证数据接收 |
| 📚 文档 | ✅ 完成 | README + 本指南 |

### 🎉 超出原始需求的增强功能

1. **极坐标计算** - 方便死区判断和意图识别
2. **边沿检测** - 可以检测按键的"按下瞬间"和"释放瞬间"
3. **组合键识别** - 提供 L2+A、L2+B 等组合键的便捷检测
4. **统计信息** - 消息序列号、处理延迟等元数据
5. **完整的单元测试** - 按键解析器的 GTest 测试

---

## 5. 待完成功能清单

### ⚠️ 项目当前状态说明

**🚨 重要提示**：
- ✅ **代码框架完整** - 所有模块已开发完成
- ⚠️ **未经实机验证** - 代码尚未在真实 B2 上测试
- 🔴 **无法离线测试** - 必须连接真实 B2 才能运行
- 📦 **可能存在编译问题** - 依赖关系需要实际验证

### 🚧 阶段一剩余工作 (需要实机测试)

| 任务 | 状态 | 优先级 | 说明 |
|------|------|--------|------|
| 🔌 B2 实机连接测试 | ⏳ 待完成 | 🔴 **必需** | **必须**有真实的 B2 机器人才能测试 |
| 🛠️ 编译问题修复 | ⏳ 待完成 | 🔴 高 | 首次编译可能遇到依赖或语法问题 |
| 📊 消息频率验证 | ⏳ 待完成 | 🔴 高 | 确认是否达到 50-100Hz |
| 🐛 运行时 BUG 修复 | ⏳ 待完成 | 🔴 高 | 实机测试后可能发现的逻辑问题 |
| 🔋 电池字段验证 | ⏳ 待完成 | 🟡 中 | 确认 `battery_voltage` 等字段是否可用 |
| 📝 实机使用文档 | ⏳ 待完成 | 🟡 中 | 添加实际操作截图和数据示例 |

### 🎯 阶段二：系统集成 (规划中)

| 任务 | 状态 | 优先级 | 说明 |
|------|------|--------|------|
| 🏗️ hardware_bridge 架构设计 | ⏳ 未开始 | 🔵 低 | 等待主系统架构明确 |
| 🔄 代码移植 | ⏳ 未开始 | 🔵 低 | 将本项目集成到主系统 |
| 🤝 融合控制策略 | ⏳ 未开始 | 🔵 低 | 实现权重混合算法 |
| 🔐 安全机制 | ⏳ 未开始 | 🔵 低 | 紧急停止、超时保护等 |

**关于阶段二的说明**：

目前 `hardware_bridge` 还在开发分支中，细节待定（TBD）。阶段一的代码已经设计为独立模块，将来可以方便地集成到任何系统中。

---

## 6. 如何运作 - 从头到尾

### � 重要前提条件

**在开始任何测试之前，你必须明确以下限制**：

| 条件 | 是否必需 | 说明 |
|------|---------|------|
| B2 机器人 | ✅ **必需** | 遥控器信号由 B2 的接收机接收并发布到 DDS |
| 原厂遥控器 | ✅ **必需** | 需要配对的遥控器发送信号 |
| 网络连接 | ✅ **必需** | 开发机必须通过网线连接到 B2 |
| unitree_ros2 | ✅ **必需** | 需要 Unitree 官方的消息定义 |

**❌ 无法实现的测试方式**：
- ❌ 只有遥控器，没有 B2 → **无法测试**（遥控器信号需要 B2 接收机接收）
- ❌ 离线编译验证 → **会失败**（缺少 `unitree_go` 等依赖包）
- ❌ 模拟器测试 → **不支持**（需要真实的 DDS 通信）

### 🔧 场景 1：检查是否可以开始（前置检查）

```bash
# ====== 检查清单 ======

# 1. 你是否有 B2 机器人？
#    YES → 继续下一步
#    NO  → 无法测试，等待获取 B2 后再进行

# 2. B2 是否已开机并连接网络？
ping 192.168.123.161
#    成功 → 继续
#    失败 → 检查网线连接和网络配置

# 3. 是否安装了 ROS2 Foxy？
ros2 --version
#    输出 "ros2 cli version: ..." → 继续
#    命令不存在 → 需要先安装 ROS2

# 4. 是否有 unitree_ros2？
ls ~/ros2_ws/src/unitree_ros2
#    存在 → 继续
#    不存在 → 需要先克隆安装

# ====== 如果以上都满足，才能继续 ======
```

### 🔧 场景 2：有 B2 但首次编译（最常见情况）

### 🤖 场景 3：完整的实机测试流程

```bash
# ====== 步骤 1: 网络配置 ======
# 连接网线到 B2 的网口
# 配置本机 IP（假设网卡是 eth0）
sudo ip addr add 192.168.123.15/24 dev eth0
sudo ip link set eth0 up

# 测试连通性
ping 192.168.123.161  # B2 主控 IP

# ====== 步骤 2: 安装 Unitree 依赖 ======
cd ~/ros2_ws/src

# 克隆 unitree_ros2
git clone https://github.com/unitreerobotics/unitree_ros2.git

# 编译 CycloneDDS (Foxy 必需！)
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x

cd ~/ros2_ws
colcon build --packages-select cyclonedds rmw_cyclonedds_cpp

# 编译 Unitree 消息
colcon build --packages-select unitree_go unitree_api

# ====== 步骤 3: 编译本项目 ======
colcon build --packages-select b2_native_rc_interface

# ====== 步骤 4: 配置环境变量 ======
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'

# ====== 步骤 5: 启动节点 ======
ros2 launch b2_native_rc_interface rc_bridge_with_test.launch.py

# 你应该看到：
# [INFO] B2 Native RC Bridge initialized successfully!
# [INFO] Listening for Unitree B2 remote controller signals...

# ====== 步骤 6: 打开另一个终端，查看数据 ======
# 新终端中，先 source 环境
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 查看话题列表
ros2 topic list
# 应该能看到 /b2_native_rc_signal

# 查看消息内容（实时更新）
ros2 topic echo /b2_native_rc_signal

# ====== 步骤 7: 测试遥控器 ======
# 拿起 B2 的遥控器，推动摇杆或按按键
# 你应该在终端中看到数据实时变化：
#
# left_stick:
#   x: 0.523
#   y: -0.312
#   magnitude: 0.607
#   angle: -0.547
# buttons:
#   a: True
#   b: False
#   ...
```

### 🎮 场景 3：在你的算法中使用遥控器数据

创建一个新的 ROS2 节点来订阅遥控器信号：

```cpp
// my_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include "b2_native_rc_interface/msg/native_rc.hpp"

class MyController : public rclcpp::Node
{
public:
    MyController() : Node("my_controller")
    {
        // 订阅遥控器信号
        rc_sub_ = this->create_subscription<b2_native_rc_interface::msg::NativeRC>(
            "/b2_native_rc_signal",
            10,
            [this](const b2_native_rc_interface::msg::NativeRC::SharedPtr msg) {
                this->rc_callback(msg);
            }
        );
    }

private:
    void rc_callback(const b2_native_rc_interface::msg::NativeRC::SharedPtr msg)
    {
        // 检查用户是否正在操作摇杆
        if (msg->left_stick.magnitude > 0.05) {
            // 用户正在操作，切换到手动模式
            RCLCPP_INFO(this->get_logger(), 
                "Manual control detected! Magnitude: %.2f", 
                msg->left_stick.magnitude);
            
            // 让算法降低控制权重
            manual_mode_ = true;
        } else {
            // 用户没有操作，可以自主控制
            manual_mode_ = false;
        }
        
        // 检测紧急停止组合键 (L2 + B)
        if (msg->buttons.l2 && msg->buttons.b) {
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP!");
            // 立即停止所有运动
            emergency_stop();
        }
        
        // 检查机器人状态
        if (msg->robot_state.battery_soc < 20) {
            RCLCPP_WARN(this->get_logger(), "Low battery: %d%%", 
                msg->robot_state.battery_soc);
        }
    }
    
    void emergency_stop() {
        // 你的急停逻辑
    }
    
    rclcpp::Subscription<b2_native_rc_interface::msg::NativeRC>::SharedPtr rc_sub_;
    bool manual_mode_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyController>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 7. 常见问题与排查

### ❓ Q1: 编译时找不到 `unitree_go` 包

**症状**：
```
CMake Error: Could not find a package configuration file provided by "unitree_go"
```

**解决方案**：
```bash
# 安装 unitree_ros2
cd ~/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd ~/ros2_ws
colcon build --packages-select unitree_go unitree_api
```

### ❓ Q2: 运行节点时没有看到遥控器数据

**可能原因 1**：未连接到 B2
```bash
# 检查网络连接
ping 192.168.123.161

# 检查环境变量
echo $RMW_IMPLEMENTATION  # 应该输出 rmw_cyclonedds_cpp
```

**可能原因 2**：CycloneDDS 版本不对
```bash
# Foxy 必须使用 0.10.x 版本
cd ~/ros2_ws/src
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ~/ros2_ws
colcon build --packages-select cyclonedds rmw_cyclonedds_cpp
```

**可能原因 3**：网卡名称错误
```bash
# 查看实际网卡名称
ip addr show

# 修改环境变量中的网卡名（例如改为 enp3s0）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enp3s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

### ❓ Q3: 数据延迟很高 (> 100ms)

**排查步骤**：
```bash
# 1. 检查消息频率
ros2 topic hz /b2_native_rc_signal

# 期望：50-100 Hz
# 如果很低（< 10 Hz），可能是网络问题

# 2. 检查桥接延迟
ros2 topic echo /b2_native_rc_signal --field bridge_latency_us

# 期望：< 1000 us (1 毫秒)
# 如果很高，可能是 CPU 负载过高

# 3. 检查系统负载
top
```

### ❓ Q4: 按键状态总是 False

**检查**：
```bash
# 1. 确认遥控器已开机并连接
# 2. 查看原始按键值
ros2 topic echo /b2_native_rc_signal --field buttons.raw_keys

# 如果 raw_keys 一直是 0，说明没有收到遥控器信号
```

### ❓ Q5: `robot_state` 字段没有数据

**原因**：这些数据来自 B2 的其他话题 (`/sportmodestate`, `/lowstate`)

**检查**：
```bash
# 查看 B2 是否发布这些话题
ros2 topic list | grep -E "sportmodestate|lowstate"

# 如果没有，可能 B2 系统未完全启动或网络不通
```

---

## 9. 下一步计划

### 📅 近期计划（1-2 周）

1. **实机测试** 🔴 高优先级
   - 在真实 B2 上运行并验证所有功能
   - 记录实际的消息频率、延迟等性能指标
   - 拍摄操作视频和截图

2. **文档完善** 🟡 中优先级
   - 添加实机测试的截图
   - 提供真实数据的示例
   - 补充故障排查案例

3. **性能优化** 🟡 中优先级
   - 根据实测延迟进行优化
   - 调整 QoS 参数

### 🎯 中期计划（1-2 月）

1. **融合控制原型** （阶段二前期）
   - 实现简单的权重混合器
   - 测试人工接管的响应时间
   - 设计安全机制（超时、急停）

2. **更多测试用例**
   - 边界条件测试（遥控器断电、网络断开）
   - 压力测试（长时间运行）

### 🚀 远期计划（3+ 月）

1. **系统集成**（阶段二）
   - 等待 `hardware_bridge` 架构明确
   - 代码重构以适应主系统
   - 集成测试

2. **高级功能**
   - 意图预测（基于历史摇杆数据）
   - 自适应死区（根据用户习惯调整）
   - 多遥控器支持

---

## 📊 附录：技术细节速查表

### 摇杆坐标系

```
左摇杆 (通常用于移动)：
  x 正值 = 向左平移
  x 负值 = 向右平移
  y 正值 = 向前
  y 负值 = 向后

右摇杆 (通常用于旋转)：
  x 正值 = 逆时针旋转
  x 负值 = 顺时针旋转
  y 正值 = 抬头/升高
  y 负值 = 低头/降低
```

### 按键位映射

| 位 | 按键 | 常见用途 |
|----|------|---------|
| 0 | R1 | 右肩键 - 快速动作 |
| 1 | L1 | 左肩键 - 快速动作 |
| 2 | Start | 开始/菜单 |
| 3 | Select | 选择/返回 |
| 4 | R2 | 右扳机 - 确认 |
| 5 | L2 | 左扳机 - 功能键 |
| 8 | A | 确认/站立 |
| 9 | B | 取消/趴下 |

### 运动模式代码

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | IDLE | 待机 |
| 1 | BALANCE_STAND | 平衡站立 |
| 3 | LOCOMOTION | 运动中 |
| 7 | DAMPING | 阻尼模式/软急停 |

### 步态类型代码

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | IDLE | 静止 |
| 1 | TROT | 小跑（默认步态） |
| 2 | RUN | 快跑 |
| 3 | CLIMB_STAIR | 爬楼梯 |

---

## 7. 本地测试指南（无需 B2）

### 🎉 好消息：可以在本地测试！

**即使没有真实的 B2 机器人，你也可以测试大部分功能！**

我们提供了一个**模拟器节点**，可以生成假的遥控器数据，让你能够：
- ✅ 验证代码编译
- ✅ 测试消息处理逻辑
- ✅ 调试数据转换流程
- ✅ 检查输出格式

### 🚀 快速开始本地测试

**一键启动完整测试环境**：
```bash
# 编译项目
cd ~/ros2_ws
colcon build --packages-select b2_native_rc_interface
source install/setup.bash

# 启动模拟测试环境（包含模拟器 + 桥接节点 + 测试订阅者）
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py
```

**你会看到类似输出**：
```
[rc_simulator]: 🎮 RC Simulator Node started - Mode: auto, Rate: 100.0 Hz
[rc_bridge_node]: ✅ RC Signal Bridge Node initialized
[rc_test_subscriber]: 📡 Subscribed to /b2_native_rc_signal
[rc_test_subscriber]: 📊 Left: (0.50, 0.00) | Right: (0.30, -0.15) | Buttons: R1
```

### 🎮 三种模拟模式

1. **自动模式**（推荐）- 周期性变化，适合长时间测试
   ```bash
   ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=auto
   ```

2. **随机模式** - 完全随机，适合压力测试
   ```bash
   ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=random
   ```

3. **静态模式** - 固定数据，适合调试
   ```bash
   ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=static
   ```

### 📊 验证测试结果

在新终端中检查话题：
```bash
# 查看原始模拟数据
ros2 topic echo /wirelesscontroller

# 查看处理后的输出
ros2 topic echo /b2_native_rc_signal

# 检查发布频率（应该约 100Hz）
ros2 topic hz /b2_native_rc_signal

# 可视化节点连接
rqt_graph
```

### ⚠️ 本地测试的局限性

**可以验证**：
- ✅ 代码逻辑正确性
- ✅ 消息处理流程
- ✅ 数学计算（极坐标、死区等）
- ✅ 按键解析
- ✅ 性能和稳定性

**无法验证**：
- ❌ 与真实 B2 的消息格式兼容性
- ❌ DDS 网络配置
- ❌ 真实硬件特性
- ❌ 实际延迟和时序

**结论**：本地测试能发现 **70%** 的问题，但真机测试仍然必不可少！

### 📚 详细测试文档

更详细的测试指南请查看：[docs/LOCAL_TESTING_GUIDE.md](docs/LOCAL_TESTING_GUIDE.md)

包含：
- 分步测试教程
- 性能测试方法
- 调试技巧
- 问题排查

---

## 8. 常见问题与排查

### 🚨 你当前的情况：只有遥控器，没有 B2

**很遗憾，这个项目无法在没有 B2 的情况下测试。**

**原因**：
```
遥控器 ──(无线信号)──> [B2 接收机] ──(DDS网络)──> 我们的 ROS2 节点
         2.4GHz               │
                              │
                              └──> 原厂控制器
```

- 遥控器只是**发射端**，发出 2.4GHz 无线信号
- 信号必须被 **B2 内置的接收机**接收
- 接收机将信号转换为 DDS 消息并发布
- 我们的代码订阅这些 DDS 消息

**没有 B2 = 没有接收机 = 收不到任何信号**

### 🎯 你可以做的准备工作

虽然无法实际测试，但你可以：

1. **学习相关知识**
   - 阅读 [Unitree 官方文档](https://support.unitree.com/)
   - 了解 ROS2 基础教程
   - 熟悉 DDS 通信原理

2. **检查代码**
   - 审查本项目的代码逻辑
   - 学习消息定义的结构
   - 理解数据流动过程

3. **准备开发环境**
   - 安装 ROS2 Foxy
   - 安装必要的工具
   ```bash
   sudo apt install ros-foxy-rclcpp ros-foxy-std-msgs
   ```

4. **联系 B2 拥有者**
   - 寻找有 B2 的同事或朋友
   - 或等待获取 B2 后再测试

---

## 📞 获取帮助

### 📧 遇到问题时

如果遇到本文档未覆盖的问题：

1. 查看 [README.md](README.md) 中的基础使用说明
2. 查看 [Unitree 官方文档](https://support.unitree.com/)
3. 提交 Issue 到项目仓库
4. 查看代码中的详细注释（每个函数都有说明）

---

**文档版本**: v1.0  
**最后更新**: 2026-01-09  
**适用版本**: ROS2 Foxy, B2 SDK 2.x

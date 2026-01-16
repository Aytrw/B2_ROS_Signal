# B2 Native RC Interface 项目指导

## 1. 项目功能

本项目为 Unitree B2 四足机器人提供原厂遥控器信号的 ROS2 接入方案。

### 1.1 功能列表

| 功能 | 说明 |
|-----|------|
| 摇杆信号捕获 | 左/右摇杆 4 轴数据 (lx, ly, rx, ry) |
| 按键状态解析 | 16 个按键的实时状态和边沿检测 |
| 机器人状态同步 | 运动模式、步态、电池等状态 |
| 死区过滤 | 消除摇杆中心抖动 |
| 平滑滤波 | 减少信号噪声和抖动 |
| 极坐标转换 | 提供摇杆的模长和角度 |

### 1.2 输入/输出

**输入话题（来自 Unitree B2）:**

| 话题 | 消息类型 | 频率 | 内容 |
|-----|---------|------|------|
| `/wirelesscontroller` | `WirelessController` | ~100Hz | 摇杆 + 按键 |
| `/sportmodestate` | `SportModeState` | ~10Hz | 运动模式、步态 |
| `/lowstate` | `LowState` | ~10Hz | 电池状态 |

**输出话题:**

| 话题 | 消息类型 | 频率 |
|-----|---------|------|
| `/b2_native_rc_signal` | `NativeRC` | ~100Hz |

---

## 2. 消息结构

### 2.1 NativeRC (主消息)

```
NativeRC
├── header              # std_msgs/Header
├── left_stick          # JoystickState - 左摇杆
├── right_stick         # JoystickState - 右摇杆
├── buttons             # ButtonState   - 按键状态
├── robot_state         # RobotState    - 机器人状态
├── seq                 # uint32        - 消息序列号
└── bridge_latency_us   # uint32        - 处理延迟(微秒)
```

### 2.2 JoystickState

| 字段 | 类型 | 范围 | 说明 |
|-----|------|------|------|
| `x` | float32 | [-1.0, 1.0] | X轴，正值向右 |
| `y` | float32 | [-1.0, 1.0] | Y轴，正值向前 |
| `magnitude` | float32 | [0, √2] | 模长 = √(x² + y²) |
| `angle` | float32 | [-π, π] | 角度 = atan2(y, x) |

### 2.3 ButtonState

| 字段 | 位 | 说明 |
|-----|---|------|
| `r1` | 0 | 右肩键 |
| `l1` | 1 | 左肩键 |
| `start` | 2 | Start键 |
| `select` | 3 | Select键 |
| `r2` | 4 | 右扳机 |
| `l2` | 5 | 左扳机 |
| `f1` | 6 | F1功能键 |
| `f2` | 7 | F2功能键 |
| `a` | 8 | A键 |
| `b` | 9 | B键 |
| `x` | 10 | X键 |
| `y` | 11 | Y键 |
| `up` | 12 | 方向键上 |
| `right` | 13 | 方向键右 |
| `down` | 14 | 方向键下 |
| `left` | 15 | 方向键左 |
| `raw_keys` | - | 16位原始位掩码 |

### 2.4 RobotState

| 字段 | 类型 | 说明 |
|-----|------|------|
| `sport_mode` | uint8 | 运动模式 (0=Idle, 1=BalanceStand, 3=Locomotion, 7=Damping) |
| `gait_type` | uint8 | 步态类型 (0=Idle, 1=Trot, 2=Run) |
| `foot_raise_height` | float32 | 抬腿高度 (米) |
| `body_height` | float32 | 机身高度 (米) |
| `battery_soc` | uint8 | 电池电量 (%) |
| `battery_voltage` | float32 | 电池电压 (V) |
| `battery_current` | float32 | 电池电流 (A) |
| `rc_connected` | bool | 遥控器连接状态 |

---

## 3. 核心实现原理

### 3.1 按键解析 (xKeySwitchUnion)

按键数据通过 16 位位掩码传输，使用联合体进行解析：

```cpp
typedef union {
    struct {
        uint8_t R1 : 1;      // Bit 0
        uint8_t L1 : 1;      // Bit 1
        uint8_t start : 1;   // Bit 2
        uint8_t select : 1;  // Bit 3
        uint8_t R2 : 1;      // Bit 4
        uint8_t L2 : 1;      // Bit 5
        uint8_t F1 : 1;      // Bit 6
        uint8_t F2 : 1;      // Bit 7
        uint8_t A : 1;       // Bit 8
        uint8_t B : 1;       // Bit 9
        uint8_t X : 1;       // Bit 10
        uint8_t Y : 1;       // Bit 11
        uint8_t up : 1;      // Bit 12
        uint8_t right : 1;   // Bit 13
        uint8_t down : 1;    // Bit 14
        uint8_t left : 1;    // Bit 15
    } components;
    uint16_t value;
} xKeySwitchUnion;
```

**使用方法:**
```cpp
xKeySwitchUnion key;
key.value = msg->keys;  // 赋值原始数据

if (key.components.A) {
    // A键被按下
}
```

### 3.2 按键边沿检测 (Button 类)

用于检测按键的按下/松开瞬间：

```cpp
class Button {
public:
    bool pressed;     // 当前是否按下
    bool on_press;    // 是否刚按下（上升沿）
    bool on_release;  // 是否刚松开（下降沿）
    
    void update(bool current) {
        on_press = current && !pressed;
        on_release = !current && pressed;
        pressed = current;
    }
};
```

**使用场景:**
```cpp
Button A;
A.update(key.components.A);

if (A.on_press) {
    // A键刚刚按下，仅触发一次
    executeAction();
}
```

### 3.3 摇杆数据处理

#### 3.3.1 死区过滤

消除摇杆中心位置的微小抖动：

```cpp
float applyDeadZone(float value, float dead_zone) {
    return std::fabs(value) < dead_zone ? 0.0f : value;
}
```

**参数:** `dead_zone = 0.01` (官方默认值)

#### 3.3.2 平滑滤波

使用一阶低通滤波减少噪声：

```cpp
// output = output * (1 - smooth) + input * smooth
float applySmooth(float output, float input, float smooth) {
    return output * (1.0f - smooth) + input * smooth;
}
```

**参数:** `smooth = 0.03` (官方默认值)

- 值越小：越平滑，响应越慢
- 值越大：响应越快，越不平滑

#### 3.3.3 极坐标转换

```cpp
magnitude = std::sqrt(x * x + y * y);
angle = std::atan2(y, x);  // 范围 [-π, π]
```

**角度含义:**
- 0° (0 rad) = 右
- 90° (π/2 rad) = 前
- 180° (π rad) = 左
- -90° (-π/2 rad) = 后

### 3.4 数据流处理

```
WirelessController 消息到达
        ↓
记录开始时间戳
        ↓
处理左摇杆: applyDeadZone → applySmooth → 转极坐标
        ↓
处理右摇杆: applyDeadZone → applySmooth → 转极坐标
        ↓
解析按键: 位掩码 → xKeySwitchUnion → ButtonState
        ↓
构建机器人状态 (从缓存读取)
        ↓
计算处理延迟
        ↓
发布 NativeRC 消息
```

---

## 4. 配置参数

配置文件: `config/rc_bridge_params.yaml`

### 4.1 话题配置

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `rc_input_topic` | `/wirelesscontroller` | 遥控器输入话题 |
| `sport_state_topic` | `/sportmodestate` | 运动状态话题 |
| `low_state_topic` | `/lowstate` | 底层状态话题 |
| `output_topic` | `/b2_native_rc_signal` | 输出话题 |

### 4.2 超时配置

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `rc_timeout_sec` | 0.5 | 遥控器超时时间 (秒) |
| `state_timeout_sec` | 1.0 | 状态超时时间 (秒) |

### 4.3 QoS 配置

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `qos_depth` | 10 | 消息队列深度 |

### 4.4 摇杆处理配置

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `joystick_dead_zone` | 0.01 | 死区阈值 |
| `joystick_smooth` | 0.03 | 平滑系数 |
| `joystick_dead_zone_enabled` | true | 启用死区过滤 |
| `joystick_smooth_enabled` | true | 启用平滑滤波 |

---

## 5. 编译与安装

### 5.1 依赖项

| 依赖 | 版本要求 |
|-----|---------|
| ROS2 | Foxy |
| CycloneDDS | 0.10.x |
| unitree_ros2 | 最新版 (包含 unitree_go, unitree_api) |

### 5.2 编译

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 5.3 验证编译

```bash
# 检查节点
ros2 pkg executables b2_native_rc_interface

# 预期输出:
# b2_native_rc_interface rc_bridge_node
# b2_native_rc_interface rc_test_subscriber
# b2_native_rc_interface rc_simulator
# b2_native_rc_interface rc_keyboard_simulator
```

---

## 6. 使用方法

### 6.1 模拟测试（无需硬件）

```bash
# 完整模拟环境（推荐先测试）
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py

# 键盘模拟器（手动控制）
ros2 launch b2_native_rc_interface rc_keyboard_test.launch.py
```

**键盘映射:**

| 按键 | 功能 |
|-----|------|
| W/S | 左摇杆 Y轴 (前进/后退) |
| A/D | 左摇杆 X轴 (左/右) |
| I/K | 右摇杆 Y轴 |
| J/L | 右摇杆 X轴 |
| 1-8 | R1, L1, R2, L2, Start, Select, F1, F2 |
| Q/E | A, B |
| Z/C | X, Y |
| ↑↓←→ | 方向键 |
| R | 重置所有值 |
| ESC | 退出 |

### 6.2 B2 真机部署

#### 步骤 1：网络配置

开发机需与 B2 在同一网段 192.168.123.xxx

#### 步骤 2：DDS 配置

```bash
# 设置 CycloneDDS 为中间件
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 指定网卡（替换 eth0 为连接 B2 的网卡）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

> **提示:** 可将上述 export 命令添加到 `~/.bashrc` 中永久生效。

#### 步骤 3：启动

```bash
ros2 launch b2_native_rc_interface rc_bridge.launch.py
```

#### 步骤 4：验证

```bash
# 查看频率（应约 100Hz）
ros2 topic hz /b2_native_rc_signal

# 查看消息内容
ros2 topic echo /b2_native_rc_signal
```

### 6.3 命令行工具

```bash
# 查看话题列表
ros2 topic list

# 查看消息频率
ros2 topic hz /b2_native_rc_signal

# 查看消息内容
ros2 topic echo /b2_native_rc_signal

# 查看消息结构
ros2 interface show b2_native_rc_interface/msg/NativeRC
```

### 6.4 自定义配置启动

```bash
# 使用自定义配置文件
ros2 launch b2_native_rc_interface rc_bridge.launch.py \
    config_file:=/path/to/custom_config.yaml

# 修改输出话题
ros2 launch b2_native_rc_interface rc_bridge.launch.py \
    output_topic:=/my_rc_signal

# 开启调试日志
ros2 launch b2_native_rc_interface rc_bridge.launch.py \
    log_level:=debug
```

---

## 7. 二次开发示例

### 7.1 C++ 订阅示例

```cpp
#include <rclcpp/rclcpp.hpp>
#include "b2_native_rc_interface/msg/native_rc.hpp"

class MyController : public rclcpp::Node {
public:
    MyController() : Node("my_controller") {
        sub_ = this->create_subscription<b2_native_rc_interface::msg::NativeRC>(
            "/b2_native_rc_signal", 10,
            std::bind(&MyController::callback, this, std::placeholders::_1));
    }

private:
    void callback(const b2_native_rc_interface::msg::NativeRC::SharedPtr msg) {
        // 获取左摇杆数据
        float vx = msg->left_stick.y;  // 前进速度
        float vy = msg->left_stick.x;  // 横移速度
        float wz = msg->right_stick.x; // 旋转速度
        
        // 检测按键
        if (msg->buttons.a) {
            RCLCPP_INFO(this->get_logger(), "A pressed");
        }
        
        // 检测组合键 L2+A (站立解锁)
        if (msg->buttons.l2 && msg->buttons.a) {
            RCLCPP_WARN(this->get_logger(), "Stand unlock combo!");
        }
        
        // 获取机器人状态
        int mode = msg->robot_state.sport_mode;
        int battery = msg->robot_state.battery_soc;
    }

    rclcpp::Subscription<b2_native_rc_interface::msg::NativeRC>::SharedPtr sub_;
};
```

### 7.2 Python 订阅示例

```python
import rclpy
from rclpy.node import Node
from b2_native_rc_interface.msg import NativeRC

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.sub = self.create_subscription(
            NativeRC, '/b2_native_rc_signal', self.callback, 10)
    
    def callback(self, msg):
        # 获取摇杆数据
        vx = msg.left_stick.y
        vy = msg.left_stick.x
        wz = msg.right_stick.x
        
        # 检测按键
        if msg.buttons.a:
            self.get_logger().info('A pressed')
        
        # 检测组合键
        if msg.buttons.l2 and msg.buttons.a:
            self.get_logger().warn('Stand unlock combo!')
        
        # 使用极坐标
        if msg.left_stick.magnitude > 0.5:
            self.get_logger().info(f'Strong input, angle: {msg.left_stick.angle}')

def main():
    rclpy.init()
    node = MyController()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 7.3 速度映射示例

```cpp
// 将摇杆值映射到机器人速度
void mapToVelocity(const b2_native_rc_interface::msg::NativeRC& msg,
                   float& vx, float& vy, float& wz) {
    // 速度上限
    const float MAX_LINEAR = 1.5;   // m/s
    const float MAX_ANGULAR = 2.0;  // rad/s
    
    // 线性映射
    vx = msg.left_stick.y * MAX_LINEAR;
    vy = msg.left_stick.x * MAX_LINEAR;
    wz = msg.right_stick.x * MAX_ANGULAR;
    
    // 可选: 非线性映射 (更精细的低速控制)
    // vx = std::copysign(std::pow(std::fabs(msg.left_stick.y), 1.5), msg.left_stick.y) * MAX_LINEAR;
}
```

---

## 8. 常用组合键

| 组合键 | 功能 | 位掩码 |
|-------|------|--------|
| L2 + A | 站立解锁 | `0x0120` |
| L2 + B | 阻尼模式 (软急停) | `0x0220` |
| L2 + R2 + X | 特殊动作 | `0x0430` |

**检测方法:**
```cpp
// 方法1: 直接检测
if (msg->buttons.l2 && msg->buttons.a) { ... }

// 方法2: 使用位掩码
if ((msg->buttons.raw_keys & 0x0120) == 0x0120) { ... }
```

---

## 9. 故障排查

### 9.1 无消息输出

```bash
# 1. 检查话题是否存在
ros2 topic list | grep wirelesscontroller

# 2. 检查 DDS 配置
echo $RMW_IMPLEMENTATION  # 应为 rmw_cyclonedds_cpp

# 3. 检查网络连接 (在 B2 上运行)
ping 192.168.123.161
```

### 9.2 消息延迟高

```bash
# 查看延迟统计
ros2 topic echo /b2_native_rc_signal --field bridge_latency_us

# 正常值: < 100 us
# 异常值: > 1000 us
```

**解决方法:**
- 降低 `qos_depth`
- 检查 CPU 负载
- 禁用不必要的平滑滤波

### 9.3 按键无响应

```bash
# 查看原始按键值
ros2 topic echo /b2_native_rc_signal --field buttons.raw_keys

# 正常: 按键按下时值非零
# 异常: 始终为 0
```

---

## 10. 文件结构

```
B2_ROS_Signal/
├── CMakeLists.txt          # 编译配置
├── package.xml             # 包描述
├── config/
│   └── rc_bridge_params.yaml   # 运行时配置
├── include/b2_native_rc_interface/
│   ├── types.hpp               # 枚举、常量、位掩码定义
│   ├── bridge/
│   │   └── rc_signal_bridge.hpp    # 桥接器类声明
│   └── utils/
│       ├── key_parser.hpp      # 按键解析器 (xKeySwitchUnion, Button)
│       └── joystick_processor.hpp  # 摇杆处理器
├── msg/
│   ├── NativeRC.msg        # 主消息
│   ├── JoystickState.msg   # 摇杆消息
│   ├── ButtonState.msg     # 按键消息
│   └── RobotState.msg      # 机器人状态消息
├── src/
│   ├── bridge/
│   │   └── rc_signal_bridge.cpp    # 桥接器实现
│   ├── nodes/
│   │   ├── rc_bridge_node.cpp      # 主节点入口
│   │   ├── rc_test_subscriber.cpp  # 测试订阅者
│   │   ├── rc_simulator_node.cpp   # 自动模拟器
│   │   └── rc_keyboard_simulator.cpp   # 键盘模拟器
│   └── utils/
│       ├── key_parser.cpp
│       └── joystick_processor.cpp
├── launch/
│   ├── rc_bridge.launch.py         # 标准启动
│   ├── rc_bridge_with_test.launch.py   # 带测试启动
│   ├── rc_simulator.launch.py      # 模拟器启动
│   ├── rc_keyboard_test.launch.py  # 键盘测试启动
│   └── rc_full_simulation.launch.py    # 完整模拟
└── test/
    └── test_key_parser.cpp         # 单元测试
```

---

## 11. 功能状态

### 11.1 已实现功能

| 功能 | 状态 | 说明 |
|-----|------|------|
| 摇杆信号捕获 | ✅ | 4 轴数据 + 极坐标转换 |
| 按键状态解析 | ✅ | 16 个按键 + 组合键检测 |
| 死区过滤 | ✅ | 官方默认值 0.01 |
| 平滑滤波 | ✅ | 官方默认值 0.03 |
| 机器人状态同步 | ✅ | 运动模式、步态、电池电量 |
| 自动模拟器 | ✅ | 无硬件测试 |
| 键盘模拟器 | ✅ | 交互式测试 |
| 测试订阅者 | ✅ | 消息验证工具 |

### 11.2 已知限制

| 限制 | 原因 | 影响 |
|-----|------|------|
| 电池电压/电流未读取 | 需真机验证 LowState 字段结构 | `battery_voltage`, `battery_current` 为 0 |
| 遥控器电量不可用 | SDK 未暴露此信息 | `rc_battery` 固定为 255 |
| 按键边沿检测未发布 | 仅内部使用 | `on_press`/`on_release` 需自行实现 |

### 11.3 待实现功能

| 功能 | 优先级 | 说明 |
|-----|--------|------|
| 参数动态更新 | 中 | 运行时修改死区/平滑参数 |
| 诊断/健康监控 | 低 | 连接状态变化事件通知 |
| JoystickProcessor 单元测试 | 低 | 补充测试覆盖率 |
| 边沿检测消息发布 | 低 | 扩展 ButtonState 包含 on_press 字段 |

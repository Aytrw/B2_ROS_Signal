# 本地测试指南（无需 B2 硬件）

## 📋 概述

即使没有真实的 B2 机器人，你也可以在本地进行大部分开发和测试工作。本指南将教你如何：

1. ✅ 验证代码编译
2. ✅ 测试消息处理逻辑
3. ✅ 调试数据转换流程
4. ✅ 检查输出格式

⚠️ **注意**：本地测试无法验证与真实 B2 的完全兼容性，实际部署前仍需真机测试。

---

## 🎯 测试策略

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────────┐
│  模拟器节点      │      │  信号桥接节点     │      │  测试订阅者      │
│  rc_simulator   │─────▶│  rc_bridge_node  │─────▶│  rc_test_       │
│                 │      │                  │      │  subscriber     │
│ 生成假数据       │      │  处理和转换       │      │  显示结果       │
└─────────────────┘      └──────────────────┘      └─────────────────┘
     模拟 B2 发布               你的代码                 验证输出
```

---

## 🚀 快速开始

### 步骤 1：编译项目

```bash
cd ~/ros2_ws  # 或你的工作空间
colcon build --packages-select b2_native_rc_interface
source install/setup.bash
```

**预期结果**：
- ✅ 编译成功，无错误
- ⚠️ 可能有警告，记录下来以便后续修复

### 步骤 2：运行完整模拟环境

最简单的测试方法 - 一键启动所有组件：

```bash
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py
```

**你会看到**：
```
[rc_simulator]: 🎮 RC Simulator Node started - Mode: auto, Rate: 100.0 Hz
[rc_bridge_node]: ✅ RC Signal Bridge Node initialized
[rc_test_subscriber]: 📡 Subscribed to /b2_native_rc_signal
...
[rc_test_subscriber]: 📊 Left: (0.50, 0.00) | Right: (0.30, -0.15) | Buttons: R1
```

### 步骤 3：验证数据流

在新终端中检查话题：

```bash
# 查看原始模拟数据
ros2 topic echo /wirelesscontroller

# 查看处理后的输出
ros2 topic echo /b2_native_rc_signal

# 查看话题列表
ros2 topic list
```

**预期话题**：
```
/wirelesscontroller        # 模拟器发布
/sportmodestate           # 模拟器发布
/lowstate                 # 模拟器发布
/b2_native_rc_signal      # 桥接节点发布（你的输出）
```

---

## 🎮 模拟模式详解

模拟器支持三种模式，适用于不同测试场景：

### 模式 1：自动模式（auto）- 推荐用于持续测试

**特点**：
- 摇杆做周期性运动（圆周 + 正弦波）
- 按键定时触发
- 适合长时间运行观察稳定性

**启动**：
```bash
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=auto
```

**数据特征**：
```
时间 0s: LX=0.50, LY=0.00, RX=0.30, RY=0.30
时间 1s: LX=0.27, LY=0.42, RX=-0.15, RY=0.07
时间 2s: LX=-0.21, LY=0.46, RX=-0.30, RY=-0.28
...（持续变化）
```

### 模式 2：随机模式（random）- 用于压力测试

**特点**：
- 完全随机的摇杆值和按键
- 测试边界情况和异常值处理
- 适合发现潜在 bug

**启动**：
```bash
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=random
```

**数据特征**：
```
不可预测的值，如：
LX=-0.87, LY=0.32, Keys=0xA3F1
LX=0.15, LY=-0.99, Keys=0x0000
...
```

### 模式 3：静态模式（static）- 用于调试

**特点**：
- 固定不变的数据
- 方便单步调试
- 适合检查特定状态

**启动**：
```bash
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=static
```

**数据特征**：
```
始终保持：
LX=0.00, LY=0.50 (前进), Keys=0x0010 (R1按下)
```

---

## 🔧 分步测试（高级）

如果你想单独测试每个组件：

### 测试 1：只运行模拟器

```bash
# 终端 1：启动模拟器
ros2 run b2_native_rc_interface rc_simulator \
    --ros-args -p simulation_mode:=auto

# 终端 2：查看模拟数据
ros2 topic echo /wirelesscontroller
```

**验证点**：
- ✅ 话题 `/wirelesscontroller` 存在
- ✅ 数据格式正确（lx, ly, rx, ry, keys）
- ✅ 数据在合理范围内（-1.0 到 1.0）

### 测试 2：桥接节点处理模拟数据

```bash
# 终端 1：模拟器
ros2 run b2_native_rc_interface rc_simulator

# 终端 2：桥接节点
ros2 run b2_native_rc_interface rc_bridge_node \
    --ros-args --params-file config/rc_bridge_params.yaml

# 终端 3：查看输出
ros2 topic echo /b2_native_rc_signal
```

**验证点**：
- ✅ 桥接节点成功订阅 `/wirelesscontroller`
- ✅ 发布 `/b2_native_rc_signal` 话题
- ✅ 消息字段完整（left_stick, right_stick, buttons, robot_state）
- ✅ 极坐标计算正确（magnitude, angle）

### 测试 3：测试订阅者

```bash
# 终端 1 + 2：运行模拟器和桥接节点（同上）

# 终端 3：测试订阅者
ros2 run b2_native_rc_interface rc_test_subscriber
```

**验证点**：
- ✅ 按键状态正确解析
- ✅ 组合键检测工作
- ✅ 数据显示格式清晰

---

## 📊 性能测试

### 测试延迟

```bash
# 使用 ros2 topic hz 检查发布频率
ros2 topic hz /wirelesscontroller      # 应该约 100Hz
ros2 topic hz /b2_native_rc_signal     # 应该约 100Hz

# 使用 ros2 topic bw 检查带宽
ros2 topic bw /b2_native_rc_signal
```

### 测试稳定性

长时间运行（建议至少 10 分钟）：

```bash
# 启动完整环境
ros2 launch b2_native_rc_interface rc_full_simulation.launch.py

# 在另一终端监控
watch -n 1 'ros2 topic hz /b2_native_rc_signal'
```

**检查项**：
- ✅ 发布频率稳定
- ✅ 无内存泄漏（使用 `top` 或 `htop` 监控）
- ✅ 无错误或警告日志

---

## 🐛 调试技巧

### 使用 rqt_graph 可视化

```bash
rqt_graph
```

**预期图形**：
```
rc_simulator → /wirelesscontroller → rc_bridge_node → /b2_native_rc_signal → rc_test_subscriber
```

### 使用 rqt_console 查看日志

```bash
rqt_console
```

过滤日志级别：
- **INFO**：正常运行信息
- **WARN**：警告（如数据超时）
- **ERROR**：错误

### 记录数据包供后续分析

```bash
# 录制
ros2 bag record /wirelesscontroller /b2_native_rc_signal

# 回放
ros2 bag play <bag_file>
```

---

## ✅ 验证清单

在认为代码"通过本地测试"之前，确保：

- [ ] **编译成功**：无错误，警告已理解
- [ ] **话题正常**：所有预期话题都在发布
- [ ] **数据格式正确**：消息字段符合定义
- [ ] **数学计算准确**：极坐标、死区等计算无误
- [ ] **按键解析正确**：16个按键状态准确
- [ ] **性能达标**：稳定 100Hz，低延迟
- [ ] **长时间稳定**：10分钟无崩溃
- [ ] **日志清晰**：错误信息有用

---

## ⚠️ 本地测试的局限性

即使通过所有本地测试，仍需注意：

### 无法验证的部分

1. **真实消息格式**
   - 模拟器使用的是推测的消息结构
   - 真实 B2 的 `unitree_go` 消息可能有细微差异
   - 例如：BMS 字段结构、数组大小等

2. **实际网络环境**
   - 本地测试在 loopback 接口
   - 真实环境有 DDS 域、多播等复杂性

3. **硬件特性**
   - 真实遥控器的死区、噪声特征
   - 电池状态、信号强度等真实数据

4. **时序行为**
   - 真实系统的延迟和抖动
   - 多个进程的同步问题

### 真机测试前的准备

**代码修改可能性**：70%
- 消息字段名称调整
- 数据类型转换
- 边界情况处理

**配置调整可能性**：90%
- 网络接口选择
- QoS 参数微调
- DDS 配置优化

**逻辑bug可能性**：30%
- 本地测试能发现大部分逻辑错误
- 但真实环境可能暴露边界情况

---

## 🎓 下一步

### 如果本地测试全部通过

**恭喜！** 你已经完成了 70% 的工作。

**接下来**：
1. 将代码提交到 Git
2. 准备真机测试环境（网络、权限等）
3. 阅读 [真机部署指南](../README.md#部署到-b2)
4. 获取 B2 访问权限

### 如果遇到问题

**编译错误**：
- 检查依赖是否完整安装
- 查看 CMakeLists.txt 配置
- 参考 [README.md](../README.md#依赖安装)

**运行时错误**：
- 查看日志定位问题
- 使用 `gdb` 调试崩溃
- 检查参数配置文件

**数据异常**：
- 用 `ros2 topic echo` 逐级检查
- 验证数学公式实现
- 增加调试输出

---

## 📚 参考资料

- [ROS2 话题调试](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html)
- [rqt 工具集](https://docs.ros.org/en/foxy/Concepts/About-RQt.html)
- [ROS2 bag 录制](https://docs.ros.org/en/foxy/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)

---

**提示**：本地测试是持续的过程，每次修改代码后都应重新验证！

# ==============================================================================
# NativeRC.msg
# ==============================================================================
# Unitree B2 原厂遥控器信号 - 主消息结构
#
# 此消息聚合了遥控器的全部输入信号及机器人状态反馈：
# - 摇杆信号 (Joystick)
# - 按键信号 (Buttons)
# - 机器人状态 (Robot State)
#
# Topic: /b2_native_rc_signal
# 发布频率: 约 50-100 Hz (跟随原厂遥控器更新频率)
#
# 设计原则：
# 1. 被动监听 - 不拦截不修改原厂控制信号
# 2. 完整性 - 保留所有原始数据精度
# 3. 可解释性 - 提供清晰的字段注释
# ==============================================================================

# ============ 消息头 (Header) ============
std_msgs/Header header

# ============ 摇杆信号 (Joystick Signals) ============
# 左摇杆 (Left Stick)
# 通常用于: 线速度控制 (vx, vy)
# - left_stick.y -> 前进/后退
# - left_stick.x -> 左移/右移 (横向平移)
b2_native_rc_interface/JoystickState left_stick

# 右摇杆 (Right Stick)
# 通常用于: 角速度/姿态控制
# - right_stick.x -> 偏航 (Yaw) 左右旋转
# - right_stick.y -> 俯仰 (Pitch) 或机身高度
b2_native_rc_interface/JoystickState right_stick

# ============ 按键信号 (Button Signals) ============
b2_native_rc_interface/ButtonState buttons

# ============ 机器人状态 (Robot State) ============
# 来源于机器人本体反馈，非遥控器输入
b2_native_rc_interface/RobotState robot_state

# ============ 元信息 (Meta Info) ============
# 消息序列号 (用于丢包检测)
uint32 seq

# 桥接节点处理延迟 (微秒)
# 从收到DDS消息到发布ROS消息的耗时
uint32 bridge_latency_us

# ==============================================================================
# RobotState.msg
# ==============================================================================
# 机器人本体状态反馈信息（来源于 rt/sportmodestate 和 rt/lowstate）
# 这些信息不来自遥控器，而是来自机器人内部状态机
#
# ⚠️ 注意：
# - 遥控器电量目前无法从SDK获取，字段保留但值可能无效
# - 部分字段需要实机验证可用性
# ==============================================================================

# ============ 运动状态 (Motion State) ============
# 运动模式 (Sport Mode)
# 0 = Idle (待机)
# 1 = BalanceStand (平衡站立)
# 2 = Pose (姿态模式)
# 3 = Locomotion (运动中)
# 4-6 = 保留
# 7 = Damping (阻尼模式/软急停)
# 8 = RecoveryStand (恢复站立)
# 其他值请参考 Unitree 官方文档
uint8 sport_mode

# 步态类型 (Gait Type)
# 0 = Idle
# 1 = Trot (小跑)
# 2 = Run (奔跑)
# 3 = Climb Stair (爬楼梯)
# 4 = Forward Down Stair (下楼梯)
# 9 = Adjust (调整)
uint8 gait_type

# 抬腿高度 (Foot Raise Height)
# 单位: 米
# 范围: 通常 0.06 ~ 0.12
float32 foot_raise_height

# 机身高度 (Body Height)
# 单位: 米
# B2 默认约 0.32m
float32 body_height

# ============ 电源状态 (Power State) ============
# 机器人电池电量 (Robot Battery SOC)
# 范围: 0 - 100 (%)
# 来源: rt/lowstate -> bms_state
uint8 battery_soc

# 电池电压 (Battery Voltage)
# 单位: 伏特 (V)
float32 battery_voltage

# 电池电流 (Battery Current)
# 单位: 安培 (A)
# 正值: 放电, 负值: 充电
float32 battery_current

# ============ 遥控器连接状态 (RC Connection) ============
# 遥控器连接状态
# True = 最近收到遥控器数据包 (< timeout)
# False = 遥控器数据超时
bool rc_connected

# 遥控器电量 (RC Battery)
# ⚠️ 此字段可能无法从SDK获取
# 范围: 0 - 100 (%)
# 值为 255 表示无法获取
uint8 rc_battery

# ============ 时间戳 (Timestamp) ============
# 最后一次收到遥控器数据的时间
builtin_interfaces/Time last_rc_timestamp

# 最后一次收到运动状态的时间
builtin_interfaces/Time last_state_timestamp

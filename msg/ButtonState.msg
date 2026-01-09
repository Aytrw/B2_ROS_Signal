# ==============================================================================
# ButtonState.msg
# ==============================================================================
# 遥控器所有按键的状态
# 每个按键对应一个bool变量：
#   - True  = 按下 (Pressed)
#   - False = 释放 (Released)
#
# 注意：组合键逻辑（如 L2+A 解锁站立）应在上层算法中判断
# ==============================================================================

# ============ 肩键 / 扳机键 (Shoulder / Trigger Buttons) ============
bool r1         # 右肩键 (Right Bumper)
bool l1         # 左肩键 (Left Bumper)
bool r2         # 右扳机键 (Right Trigger) - 数字量
bool l2         # 左扳机键 (Left Trigger) - 数字量

# ============ 功能键 (Function Buttons) ============
bool start      # 开始键 (Start / Menu)
bool select     # 选择键 (Select / Back)
bool f1         # 功能键 F1 (可能为遥控器特有)
bool f2         # 功能键 F2 (可能为遥控器特有)

# ============ 动作键 (Action Buttons - ABXY) ============
bool a          # A键 (通常为确认/执行)
bool b          # B键 (通常为取消/返回)
bool x          # X键
bool y          # Y键

# ============ 方向键 / 十字键 (D-Pad) ============
bool up         # 方向键 上
bool down       # 方向键 下
bool left       # 方向键 左
bool right      # 方向键 右

# ============ 原始数据 (Raw Data) ============
# 16位按键位掩码，用于自定义解析或调试
# Bit 0-7:  R1, L1, Start, Select, R2, L2, F1, F2
# Bit 8-15: A, B, X, Y, Up, Right, Down, Left
uint16 raw_keys

/**
 * @file types.hpp
 * @brief 公共类型定义
 * 
 * 本文件定义了与 Unitree 官方 SDK 兼容的类型和常量。
 * 命名规范遵循官方文档: https://support.unitree.com/home/zh/B2_developer/Get%20Remote%20Control%20Status
 * 
 * @author B2 Native RC Interface Team
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#ifndef B2_NATIVE_RC_INTERFACE__TYPES_HPP_
#define B2_NATIVE_RC_INTERFACE__TYPES_HPP_

#include <cstdint>

namespace b2_native_rc_interface
{

// ============================================================================
// 运动模式枚举 (Sport Mode Enumeration)
// ============================================================================
/**
 * @brief 运动模式枚举
 * 
 * 来源于 Unitree B2 SDK，定义了机器人的各种运动模式。
 * 对应 sportmodestate 消息中的 mode 字段。
 */
enum class SportMode : uint8_t
{
    IDLE = 0,              ///< 待机模式
    BALANCE_STAND = 1,     ///< 平衡站立模式
    POSE = 2,              ///< 姿态模式
    LOCOMOTION = 3,        ///< 运动模式
    RESERVED_4 = 4,        ///< 保留
    LIE_DOWN = 5,          ///< 趴下模式
    JOINT_LOCK = 6,        ///< 关节锁定
    DAMPING = 7,           ///< 阻尼模式（软急停）
    RECOVERY_STAND = 8,    ///< 恢复站立
    RESERVED_9 = 9,        ///< 保留
    SIT = 10,              ///< 坐下模式
    FRONT_FLIP = 11,       ///< 前翻
    FRONT_JUMP = 12,       ///< 前跳
    FRONT_POUNCE = 13      ///< 前扑
};

// ============================================================================
// 步态类型枚举 (Gait Type Enumeration)
// ============================================================================
/**
 * @brief 步态类型枚举
 * 
 * 定义了机器人的各种步态类型。
 * 对应 sportmodestate 消息中的 gait_type 字段。
 */
enum class GaitType : uint8_t
{
    IDLE = 0,              ///< 待机
    TROT = 1,              ///< 小跑
    RUN = 2,               ///< 奔跑
    CLIMB_STAIR = 3,       ///< 上楼梯
    FORWARD_DOWN_STAIR = 4,///< 下楼梯
    RESERVED_5 = 5,        ///< 保留
    RESERVED_6 = 6,        ///< 保留
    RESERVED_7 = 7,        ///< 保留
    RESERVED_8 = 8,        ///< 保留
    ADJUST = 9             ///< 调整步态
};

// ============================================================================
// 按键位定义 (Key Bits Definition)
// ============================================================================
/**
 * @brief 按键位掩码定义
 * 
 * 用于通过位运算检测按键状态。
 * 与官方 xKeySwitchUnion 联合体的位布局一致。
 */
namespace KeyBits
{
    constexpr uint16_t R1     = 1 << 0;   ///< Bit 0: 右肩键
    constexpr uint16_t L1     = 1 << 1;   ///< Bit 1: 左肩键
    constexpr uint16_t START  = 1 << 2;   ///< Bit 2: Start键
    constexpr uint16_t SELECT = 1 << 3;   ///< Bit 3: Select键
    constexpr uint16_t R2     = 1 << 4;   ///< Bit 4: 右扳机
    constexpr uint16_t L2     = 1 << 5;   ///< Bit 5: 左扳机
    constexpr uint16_t F1     = 1 << 6;   ///< Bit 6: F1功能键
    constexpr uint16_t F2     = 1 << 7;   ///< Bit 7: F2功能键
    constexpr uint16_t A      = 1 << 8;   ///< Bit 8: A键
    constexpr uint16_t B      = 1 << 9;   ///< Bit 9: B键
    constexpr uint16_t X      = 1 << 10;  ///< Bit 10: X键
    constexpr uint16_t Y      = 1 << 11;  ///< Bit 11: Y键
    constexpr uint16_t UP     = 1 << 12;  ///< Bit 12: 方向键上
    constexpr uint16_t RIGHT  = 1 << 13;  ///< Bit 13: 方向键右
    constexpr uint16_t DOWN   = 1 << 14;  ///< Bit 14: 方向键下
    constexpr uint16_t LEFT   = 1 << 15;  ///< Bit 15: 方向键左
}

// ============================================================================
// 组合键定义 (Key Combinations Definition)
// ============================================================================
/**
 * @brief 常用组合键定义
 * 
 * 定义了常用的组合键位掩码，用于快速检测组合键状态。
 */
namespace KeyCombos
{
    constexpr uint16_t STAND_LOCK = KeyBits::L2 | KeyBits::A;     ///< L2+A: 站立解锁
    constexpr uint16_t DAMPING = KeyBits::L2 | KeyBits::B;        ///< L2+B: 阻尼模式
    constexpr uint16_t SPECIAL_ACTION = KeyBits::L2 | KeyBits::R2 | KeyBits::X;  ///< L2+R2+X: 特殊动作
}

// ============================================================================
// 常量定义 (Constants Definition)
// ============================================================================
/**
 * @brief 数学常量和默认参数
 * 
 * 默认值参考 Unitree 官方例程:
 * - dead_zone = 0.01 (官方默认值)
 * - smooth = 0.03 (官方默认值)
 */
namespace Constants
{
    // 数学常量
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;
    constexpr float HALF_PI = PI / 2.0f;
    
    // 摇杆处理参数 - 与官方 Gamepad 类一致
    // 参考: https://support.unitree.com/home/zh/B2_developer/Get%20Remote%20Control%20Status
    constexpr float DEFAULT_DEAD_ZONE = 0.01f;   ///< 官方默认死区值
    constexpr float DEFAULT_SMOOTH = 0.03f;      ///< 官方默认平滑系数
    
    // 摇杆范围
    constexpr float MAX_JOYSTICK_MAGNITUDE = 1.414213562f;  ///< sqrt(2)，对角线最大值
}

}  // namespace b2_native_rc_interface

#endif  // B2_NATIVE_RC_INTERFACE__TYPES_HPP_

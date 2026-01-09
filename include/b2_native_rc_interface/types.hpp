/**
 * @file types.hpp
 * @brief 公共类型定义
 * 
 * @author Your Name
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#ifndef B2_NATIVE_RC_INTERFACE__TYPES_HPP_
#define B2_NATIVE_RC_INTERFACE__TYPES_HPP_

#include <cstdint>

namespace b2_native_rc_interface
{

/**
 * @brief 运动模式枚举
 */
enum class SportMode : uint8_t
{
    IDLE = 0,
    BALANCE_STAND = 1,
    POSE = 2,
    LOCOMOTION = 3,
    RESERVED_4 = 4,
    LIE_DOWN = 5,
    JOINT_LOCK = 6,
    DAMPING = 7,
    RECOVERY_STAND = 8,
    RESERVED_9 = 9,
    SIT = 10,
    FRONT_FLIP = 11,
    FRONT_JUMP = 12,
    FRONT_POUNCE = 13
};

/**
 * @brief 步态类型枚举
 */
enum class GaitType : uint8_t
{
    IDLE = 0,
    TROT = 1,
    RUN = 2,
    CLIMB_STAIR = 3,
    FORWARD_DOWN_STAIR = 4,
    RESERVED_5 = 5,
    RESERVED_6 = 6,
    RESERVED_7 = 7,
    RESERVED_8 = 8,
    ADJUST = 9
};

/**
 * @brief 按键位定义
 */
namespace KeyBits
{
    constexpr uint16_t R1     = 1 << 0;
    constexpr uint16_t L1     = 1 << 1;
    constexpr uint16_t START  = 1 << 2;
    constexpr uint16_t SELECT = 1 << 3;
    constexpr uint16_t R2     = 1 << 4;
    constexpr uint16_t L2     = 1 << 5;
    constexpr uint16_t F1     = 1 << 6;
    constexpr uint16_t F2     = 1 << 7;
    constexpr uint16_t A      = 1 << 8;
    constexpr uint16_t B      = 1 << 9;
    constexpr uint16_t X      = 1 << 10;
    constexpr uint16_t Y      = 1 << 11;
    constexpr uint16_t UP     = 1 << 12;
    constexpr uint16_t RIGHT  = 1 << 13;
    constexpr uint16_t DOWN   = 1 << 14;
    constexpr uint16_t LEFT   = 1 << 15;
}

/**
 * @brief 组合键定义
 */
namespace KeyCombos
{
    constexpr uint16_t STAND_LOCK = KeyBits::L2 | KeyBits::A;
    constexpr uint16_t DAMPING = KeyBits::L2 | KeyBits::B;
    constexpr uint16_t SPECIAL_ACTION = KeyBits::L2 | KeyBits::R2 | KeyBits::X;
}

/**
 * @brief 数学常量
 */
namespace Constants
{
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;
    constexpr float HALF_PI = PI / 2.0f;
    
    constexpr float DEFAULT_DEADZONE = 0.05f;
    constexpr float DEFAULT_SMOOTH = 0.1f;
    
    constexpr float MAX_JOYSTICK_MAGNITUDE = 1.414213562f;  // sqrt(2)
}

}  // namespace b2_native_rc_interface

#endif  // B2_NATIVE_RC_INTERFACE__TYPES_HPP_

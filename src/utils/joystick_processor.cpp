/**
 * @file joystick_processor.cpp
 * @brief 摇杆处理器实现 - 与 Unitree 官方 Gamepad 类兼容
 * 
 * 实现了官方 Gamepad 类的摇杆处理逻辑：
 * - 死区过滤: fabs(value) < dead_zone ? 0.0 : value
 * - 平滑滤波: output = output * (1 - smooth) + input * smooth
 * 
 * 参考文档: https://support.unitree.com/home/zh/B2_developer/Get%20Remote%20Control%20Status
 * 
 * @author B2 Native RC Interface Team
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#include "b2_native_rc_interface/utils/joystick_processor.hpp"

namespace b2_native_rc_interface
{

// ============================================================================
// 构造函数
// ============================================================================
JoystickProcessor::JoystickProcessor(const JoystickConfig& config)
    : config_(config)
{
    reset();
}

// ============================================================================
// 重置处理器状态
// ============================================================================
void JoystickProcessor::reset()
{
    // 清零上一次数据，重置平滑历史
    last_data_ = JoystickData{};
}

// ============================================================================
// 处理摇杆原始数据 - 官方 Gamepad::Update() 逻辑
// ============================================================================
JoystickData JoystickProcessor::process(float raw_x, float raw_y)
{
    JoystickData data;
    
    // Step 1: 限制输入范围到 [-1.0, 1.0]
    float x = clamp(raw_x, -1.0f, 1.0f);
    float y = clamp(raw_y, -1.0f, 1.0f);
    
    // Step 2: 应用死区过滤 (官方实现)
    // 官方公式: std::fabs(key_msg.lx()) < dead_zone ? 0.0 : key_msg.lx()
    if (config_.enable_dead_zone) {
        x = applyDeadZone(x, config_.dead_zone);
        y = applyDeadZone(y, config_.dead_zone);
    }
    
    // Step 3: 应用平滑滤波 (官方实现)
    // 官方公式: lx = lx * (1 - smooth) + filtered_lx * smooth
    if (config_.enable_smooth) {
        x = applySmooth(x, last_data_.x, config_.smooth);
        y = applySmooth(y, last_data_.y, config_.smooth);
    }
    
    // Step 4: 计算极坐标
    float magnitude = calculateMagnitude(x, y);
    float angle = calculateAngle(x, y);
    
    // Step 5: 填充输出数据
    data.x = x;
    data.y = y;
    data.magnitude = magnitude;
    data.angle = angle;
    
    // Step 6: 保存当前状态（用于下次平滑计算）
    last_data_ = data;
    
    return data;
}

// ============================================================================
// 计算极坐标模长
// ============================================================================
float JoystickProcessor::calculateMagnitude(float x, float y)
{
    return std::sqrt(x * x + y * y);
}

// ============================================================================
// 计算极坐标角度
// ============================================================================
float JoystickProcessor::calculateAngle(float x, float y)
{
    return std::atan2(y, x);
}

// ============================================================================
// 应用死区 - 官方实现
// ============================================================================
float JoystickProcessor::applyDeadZone(float value, float dead_zone)
{
    // 官方公式: std::fabs(value) < dead_zone ? 0.0 : value
    if (std::fabs(value) < dead_zone) {
        return 0.0f;
    }
    return value;
}

// ============================================================================
// 限制值到范围内
// ============================================================================
float JoystickProcessor::clamp(float value, float min_val, float max_val)
{
    return std::max(min_val, std::min(max_val, value));
}

// ============================================================================
// 应用平滑滤波 - 官方实现
// ============================================================================
float JoystickProcessor::applySmooth(float current, float previous, float smooth)
{
    // 官方公式: output = previous * (1 - smooth) + current * smooth
    // 这是一个低通滤波器/指数移动平均
    // smooth 值越小，平滑效果越强，响应越慢
    // smooth 值越大，响应越快，但可能有抖动
    return previous * (1.0f - smooth) + current * smooth;
}

// ============================================================================
// 判断摇杆是否处于中立位置
// ============================================================================
bool JoystickProcessor::isNeutral(const JoystickData& data, float threshold)
{
    return data.magnitude < threshold;
}

}  // namespace b2_native_rc_interface

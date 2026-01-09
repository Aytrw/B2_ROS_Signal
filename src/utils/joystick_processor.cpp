/**
 * @file joystick_processor.cpp
 * @brief 摇杆处理器实现
 * 
 * @author Your Name
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#include "b2_native_rc_interface/utils/joystick_processor.hpp"

namespace b2_native_rc_interface
{

JoystickProcessor::JoystickProcessor(const JoystickConfig& config)
    : config_(config)
{
    reset();
}

void JoystickProcessor::reset()
{
    last_data_ = JoystickData{};
}

JoystickData JoystickProcessor::process(float raw_x, float raw_y)
{
    JoystickData data;
    
    // Step 1: 限制范围
    float x = clamp(raw_x, -1.0f, 1.0f);
    float y = clamp(raw_y, -1.0f, 1.0f);
    
    // Step 2: 计算极坐标（在死区处理前）
    float magnitude = calculateMagnitude(x, y);
    float angle = calculateAngle(x, y);
    
    // Step 3: 应用死区
    if (config_.enable_deadzone) {
        if (magnitude < config_.deadzone) {
            x = 0.0f;
            y = 0.0f;
            magnitude = 0.0f;
            // angle 保持最后有效值（或设为0）
        } else {
            // 可选：重新映射死区外的值到 [0, 1]
            // magnitude = (magnitude - config_.deadzone) / (1.0f - config_.deadzone);
        }
    }
    
    // Step 4: 应用平滑（可选）
    if (config_.enable_smooth) {
        x = applySmooth(x, last_data_.x, config_.smooth);
        y = applySmooth(y, last_data_.y, config_.smooth);
        magnitude = calculateMagnitude(x, y);
        angle = calculateAngle(x, y);
    }
    
    // Step 5: 填充数据
    data.x = x;
    data.y = y;
    data.magnitude = magnitude;
    data.angle = angle;
    
    // Step 6: 保存状态
    last_data_ = data;
    
    return data;
}

float JoystickProcessor::calculateMagnitude(float x, float y)
{
    return std::sqrt(x * x + y * y);
}

float JoystickProcessor::calculateAngle(float x, float y)
{
    return std::atan2(y, x);
}

float JoystickProcessor::applyDeadzone(float value, float deadzone)
{
    if (std::fabs(value) < deadzone) {
        return 0.0f;
    }
    return value;
}

float JoystickProcessor::clamp(float value, float min_val, float max_val)
{
    return std::max(min_val, std::min(max_val, value));
}

float JoystickProcessor::applySmooth(float current, float previous, float smooth)
{
    // 指数移动平均 (EMA)
    // smooth = 1.0 表示无平滑（直接使用当前值）
    // smooth = 0.0 表示完全不更新（使用上一次值）
    return previous * (1.0f - smooth) + current * smooth;
}

bool JoystickProcessor::isNeutral(const JoystickData& data, float threshold)
{
    return data.magnitude < threshold;
}

}  // namespace b2_native_rc_interface

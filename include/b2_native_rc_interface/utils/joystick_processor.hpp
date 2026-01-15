/**
 * @file joystick_processor.hpp
 * @brief 摇杆数据处理器 - 与 Unitree 官方 Gamepad 类兼容
 * 
 * 提供摇杆数据的预处理功能：
 * - 死区过滤 (dead_zone)
 * - 平滑滤波 (smooth)
 * - 笛卡尔坐标到极坐标的转换
 * 
 * 平滑算法采用官方实现:
 *   output = output * (1 - smooth) + input * smooth
 * 其中 smooth 默认值为 0.03
 * 
 * 参考文档: https://support.unitree.com/home/zh/B2_developer/Get%20Remote%20Control%20Status
 * 
 * @author B2 Native RC Interface Team
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#ifndef B2_NATIVE_RC_INTERFACE__JOYSTICK_PROCESSOR_HPP_
#define B2_NATIVE_RC_INTERFACE__JOYSTICK_PROCESSOR_HPP_

#include <cmath>
#include <algorithm>
#include "b2_native_rc_interface/types.hpp"

namespace b2_native_rc_interface
{

// ============================================================================
// 摇杆数据结构 (Joystick Data Structure)
// ============================================================================
/**
 * @brief 摇杆数据结构
 * 
 * 包含笛卡尔坐标和极坐标两种表示方式：
 * - 笛卡尔坐标 (x, y): 用于直接速度控制
 * - 极坐标 (magnitude, angle): 用于意图识别和死区判断
 */
struct JoystickData
{
    // 笛卡尔坐标 (Cartesian Coordinates)
    float x = 0.0f;         ///< X轴值 [-1.0, 1.0]，正值向右
    float y = 0.0f;         ///< Y轴值 [-1.0, 1.0]，正值向前
    
    // 极坐标 (Polar Coordinates)
    float magnitude = 0.0f; ///< 模长 [0.0, √2]，sqrt(x² + y²)
    float angle = 0.0f;     ///< 角度 [-π, π]，atan2(y, x)
};

// ============================================================================
// 摇杆处理器配置 (Joystick Config)
// ============================================================================
/**
 * @brief 摇杆处理器配置 - 参数默认值与官方 Gamepad 类一致
 * 
 * 官方默认值:
 * - dead_zone = 0.01
 * - smooth = 0.03
 */
struct JoystickConfig
{
    /// 死区阈值 (官方默认 0.01)
    /// 当 |value| < dead_zone 时，输出为 0
    float dead_zone = Constants::DEFAULT_DEAD_ZONE;
    
    /// 平滑系数 (官方默认 0.03)
    /// 公式: output = output * (1 - smooth) + input * smooth
    /// 值越小越平滑，值越大响应越快
    float smooth = Constants::DEFAULT_SMOOTH;
    
    /// 是否启用死区过滤
    bool enable_dead_zone = true;
    
    /// 是否启用平滑滤波
    bool enable_smooth = true;
};

// ============================================================================
// 摇杆数据处理器类 (Joystick Processor Class)
// ============================================================================
/**
 * @brief 摇杆数据处理器类 - 实现官方 Gamepad 类的摇杆处理逻辑
 * 
 * 官方处理流程:
 * 1. 死区过滤: if |raw_value| < dead_zone then output = 0 else output = raw_value
 * 2. 平滑滤波: output = prev_output * (1 - smooth) + input * smooth
 * 
 * 使用示例:
 * @code
 * JoystickProcessor processor;
 * processor.setConfig({.dead_zone = 0.01f, .smooth = 0.03f});
 * 
 * // 每帧调用
 * auto data = processor.process(joystick.lx(), joystick.ly());
 * @endcode
 */
class JoystickProcessor
{
public:
    /**
     * @brief 构造函数
     * @param config 处理器配置（使用官方默认值）
     */
    explicit JoystickProcessor(const JoystickConfig& config = JoystickConfig());
    
    ~JoystickProcessor() = default;
    
    /**
     * @brief 处理摇杆原始数据 - 按官方 Gamepad::Update() 逻辑
     * 
     * 处理流程：
     * 1. 死区过滤: fabs(raw) < dead_zone ? 0.0 : raw
     * 2. 平滑滤波: output = output * (1 - smooth) + filtered * smooth
     * 3. 计算极坐标: magnitude = sqrt(x² + y²), angle = atan2(y, x)
     * 
     * @param raw_x 原始X轴值 [-1.0, 1.0]
     * @param raw_y 原始Y轴值 [-1.0, 1.0]
     * @return 处理后的摇杆数据
     */
    JoystickData process(float raw_x, float raw_y);
    
    /**
     * @brief 获取上一次处理的数据
     * @return 上一次的摇杆数据
     */
    const JoystickData& getLastData() const { return last_data_; }
    
    /**
     * @brief 更新配置
     * @param config 新配置
     */
    void setConfig(const JoystickConfig& config) { config_ = config; }
    
    /**
     * @brief 获取当前配置
     * @return 当前配置
     */
    const JoystickConfig& getConfig() const { return config_; }
    
    /**
     * @brief 重置处理器状态（清零平滑历史）
     */
    void reset();
    
    // ========== 静态工具函数 ==========
    
    /**
     * @brief 计算极坐标模长
     * @param x X轴值
     * @param y Y轴值
     * @return 模长 sqrt(x² + y²)
     */
    static float calculateMagnitude(float x, float y);
    
    /**
     * @brief 计算极坐标角度
     * @param x X轴值
     * @param y Y轴值
     * @return 角度 atan2(y, x)，单位弧度
     */
    static float calculateAngle(float x, float y);
    
    /**
     * @brief 应用死区 - 官方实现
     * 
     * 官方公式: fabs(value) < dead_zone ? 0.0 : value
     * 
     * @param value 输入值
     * @param dead_zone 死区阈值
     * @return 处理后的值
     */
    static float applyDeadZone(float value, float dead_zone);
    
    /**
     * @brief 限制值到范围内
     * @param value 输入值
     * @param min_val 最小值
     * @param max_val 最大值
     * @return 限制后的值
     */
    static float clamp(float value, float min_val, float max_val);
    
    /**
     * @brief 判断摇杆是否处于中立位置
     * @param data 摇杆数据
     * @param threshold 阈值
     * @return 是否中立
     */
    static bool isNeutral(const JoystickData& data, float threshold = 0.01f);

private:
    JoystickConfig config_;   ///< 处理器配置
    JoystickData last_data_;  ///< 上一次处理结果（用于平滑）
    
    /**
     * @brief 应用平滑滤波 - 官方实现
     * 
     * 官方公式: output = previous * (1 - smooth) + current * smooth
     * 
     * @param current 当前值
     * @param previous 上一次值
     * @param smooth 平滑系数
     * @return 平滑后的值
     */
    static float applySmooth(float current, float previous, float smooth);
};

}  // namespace b2_native_rc_interface

#endif  // B2_NATIVE_RC_INTERFACE__JOYSTICK_PROCESSOR_HPP_

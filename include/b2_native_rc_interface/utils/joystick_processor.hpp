/**
 * @file joystick_processor.hpp
 * @brief 摇杆数据处理器
 * 
 * 提供摇杆数据的预处理功能：
 * - 笛卡尔坐标到极坐标的转换
 * - 死区过滤
 * - 平滑滤波
 * - 范围限制
 * 
 * @author Your Name
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#ifndef B2_NATIVE_RC_INTERFACE__JOYSTICK_PROCESSOR_HPP_
#define B2_NATIVE_RC_INTERFACE__JOYSTICK_PROCESSOR_HPP_

#include <cmath>
#include <algorithm>

namespace b2_native_rc_interface
{

/**
 * @brief 摇杆数据结构
 */
struct JoystickData
{
    // 笛卡尔坐标
    float x = 0.0f;         ///< X轴值 [-1.0, 1.0]
    float y = 0.0f;         ///< Y轴值 [-1.0, 1.0]
    
    // 极坐标
    float magnitude = 0.0f; ///< 模长 [0.0, 1.414]
    float angle = 0.0f;     ///< 角度 [-π, π]
};

/**
 * @brief 摇杆处理器配置
 */
struct JoystickConfig
{
    float deadzone = 0.05f;    ///< 死区阈值 (magnitude < deadzone 视为零)
    float smooth = 0.1f;       ///< 平滑系数 [0.0, 1.0], 值越小越平滑
    bool enable_deadzone = true;  ///< 是否启用死区
    bool enable_smooth = false;   ///< 是否启用平滑（原始数据不平滑）
};

/**
 * @brief 摇杆数据处理器类
 */
class JoystickProcessor
{
public:
    /**
     * @brief 构造函数
     * @param config 处理器配置
     */
    explicit JoystickProcessor(const JoystickConfig& config = JoystickConfig());
    
    ~JoystickProcessor() = default;
    
    /**
     * @brief 处理摇杆原始数据
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
     * @brief 重置处理器状态
     */
    void reset();
    
    // ========== 静态工具函数 ==========
    
    /**
     * @brief 计算极坐标模长
     * @param x X轴值
     * @param y Y轴值
     * @return 模长 sqrt(x^2 + y^2)
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
     * @brief 应用死区
     * @param value 输入值
     * @param deadzone 死区阈值
     * @return 处理后的值
     */
    static float applyDeadzone(float value, float deadzone);
    
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
    static bool isNeutral(const JoystickData& data, float threshold = 0.05f);

private:
    JoystickConfig config_;
    JoystickData last_data_;
    
    /**
     * @brief 应用平滑滤波
     * @param current 当前值
     * @param previous 上一次值
     * @param smooth 平滑系数
     * @return 平滑后的值
     */
    static float applySmooth(float current, float previous, float smooth);
};

}  // namespace b2_native_rc_interface

#endif  // B2_NATIVE_RC_INTERFACE__JOYSTICK_PROCESSOR_HPP_

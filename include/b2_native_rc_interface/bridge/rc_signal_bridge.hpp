/**
 * @file rc_signal_bridge.hpp
 * @brief 遥控器信号桥接器
 * 
 * 核心组件，负责：
 * 1. 订阅 Unitree DDS 话题 (通过 unitree_go 消息)
 * 2. 解析遥控器和机器人状态数据
 * 3. 发布 ROS2 自定义消息
 * 
 * @author Your Name
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#ifndef B2_NATIVE_RC_INTERFACE__RC_SIGNAL_BRIDGE_HPP_
#define B2_NATIVE_RC_INTERFACE__RC_SIGNAL_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>

// Unitree ROS2 Messages
#include "unitree_go/msg/wireless_controller.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"

// Custom Messages
#include "b2_native_rc_interface/msg/native_rc.hpp"
#include "b2_native_rc_interface/msg/joystick_state.hpp"
#include "b2_native_rc_interface/msg/button_state.hpp"
#include "b2_native_rc_interface/msg/robot_state.hpp"

// Utils
#include "b2_native_rc_interface/utils/key_parser.hpp"
#include "b2_native_rc_interface/utils/joystick_processor.hpp"

namespace b2_native_rc_interface
{

/**
 * @brief 桥接器配置结构
 */
struct BridgeConfig
{
    // Topic 名称
    std::string rc_input_topic = "/wirelesscontroller";
    std::string sport_state_topic = "/sportmodestate";
    std::string low_state_topic = "/lowstate";
    std::string output_topic = "/b2_native_rc_signal";
    
    // 超时设置
    double rc_timeout_sec = 0.5;           ///< 遥控器超时时间（秒）
    double state_timeout_sec = 1.0;        ///< 状态超时时间（秒）
    
    // QoS 设置
    int qos_depth = 10;
    
    // 摇杆处理
    JoystickConfig left_stick_config;
    JoystickConfig right_stick_config;
};

/**
 * @brief 遥控器信号桥接器类
 * 
 * 设计原则：
 * - 被动监听：只订阅不发布到原厂话题
 * - 线程安全：使用互斥锁保护共享数据
 * - 低延迟：回调中直接处理并发布
 */
class RCSignalBridge : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param options ROS2 节点选项
     */
    explicit RCSignalBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    ~RCSignalBridge() override = default;
    
    /**
     * @brief 获取桥接器统计信息
     */
    struct Statistics
    {
        uint64_t rc_msg_count = 0;
        uint64_t sport_msg_count = 0;
        uint64_t low_msg_count = 0;
        uint64_t published_count = 0;
        double avg_latency_us = 0.0;
    };
    
    Statistics getStatistics() const;

private:
    // ========== 初始化函数 ==========
    void declareParameters();
    void loadConfig();
    void setupSubscribers();
    void setupPublisher();
    
    // ========== 回调函数 ==========
    void wirelessControllerCallback(const unitree_go::msg::WirelessController::SharedPtr msg);
    void sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    
    // ========== 数据处理 ==========
    msg::JoystickState processJoystick(float x, float y, JoystickProcessor& processor);
    msg::ButtonState processButtons(uint16_t raw_keys);
    msg::RobotState buildRobotState();
    
    // ========== 发布函数 ==========
    void publishNativeRC();
    
    // ========== 成员变量 ==========
    
    // 配置
    BridgeConfig config_;
    
    // 订阅者
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr rc_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_sub_;
    
    // 发布者
    rclcpp::Publisher<msg::NativeRC>::SharedPtr rc_pub_;
    
    // 处理器
    std::unique_ptr<KeyParser> key_parser_;
    std::unique_ptr<JoystickProcessor> left_stick_processor_;
    std::unique_ptr<JoystickProcessor> right_stick_processor_;
    
    // 缓存的状态数据（需要线程保护）
    struct CachedState
    {
        // 运动状态
        uint8_t sport_mode = 0;
        uint8_t gait_type = 0;
        float foot_raise_height = 0.0f;
        float body_height = 0.32f;
        
        // 电源状态
        uint8_t battery_soc = 0;
        float battery_voltage = 0.0f;
        float battery_current = 0.0f;
        
        // 时间戳
        rclcpp::Time last_sport_update;
        rclcpp::Time last_low_update;
    };
    
    CachedState cached_state_;
    mutable std::mutex state_mutex_;
    
    // 遥控器连接状态
    std::atomic<bool> rc_connected_{false};
    rclcpp::Time last_rc_time_;
    
    // 统计信息
    mutable std::mutex stats_mutex_;
    Statistics stats_;
    
    // 消息序列号
    std::atomic<uint32_t> seq_{0};
};

}  // namespace b2_native_rc_interface

#endif  // B2_NATIVE_RC_INTERFACE__RC_SIGNAL_BRIDGE_HPP_

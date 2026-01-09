/**
 * @file rc_signal_bridge.cpp
 * @brief 遥控器信号桥接器实现
 * 
 * @author Your Name
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#include "b2_native_rc_interface/bridge/rc_signal_bridge.hpp"
#include <chrono>

namespace b2_native_rc_interface
{

RCSignalBridge::RCSignalBridge(const rclcpp::NodeOptions& options)
    : Node("b2_native_rc_bridge", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing B2 Native RC Signal Bridge...");
    
    // 初始化处理器
    key_parser_ = std::make_unique<KeyParser>();
    left_stick_processor_ = std::make_unique<JoystickProcessor>();
    right_stick_processor_ = std::make_unique<JoystickProcessor>();
    
    // 初始化时间戳
    last_rc_time_ = this->now();
    cached_state_.last_sport_update = this->now();
    cached_state_.last_low_update = this->now();
    
    // 声明并加载参数
    declareParameters();
    loadConfig();
    
    // 设置订阅者和发布者
    setupSubscribers();
    setupPublisher();
    
    RCLCPP_INFO(this->get_logger(), 
        "B2 Native RC Bridge initialized successfully!\n"
        "  - RC Input Topic: %s\n"
        "  - Sport State Topic: %s\n"
        "  - Low State Topic: %s\n"
        "  - Output Topic: %s",
        config_.rc_input_topic.c_str(),
        config_.sport_state_topic.c_str(),
        config_.low_state_topic.c_str(),
        config_.output_topic.c_str());
}

void RCSignalBridge::declareParameters()
{
    // Topic 参数
    this->declare_parameter<std::string>("rc_input_topic", "/wirelesscontroller");
    this->declare_parameter<std::string>("sport_state_topic", "/sportmodestate");
    this->declare_parameter<std::string>("low_state_topic", "/lowstate");
    this->declare_parameter<std::string>("output_topic", "/b2_native_rc_signal");
    
    // 超时参数
    this->declare_parameter<double>("rc_timeout_sec", 0.5);
    this->declare_parameter<double>("state_timeout_sec", 1.0);
    
    // QoS 参数
    this->declare_parameter<int>("qos_depth", 10);
    
    // 摇杆处理参数
    this->declare_parameter<double>("joystick_deadzone", 0.05);
    this->declare_parameter<bool>("joystick_deadzone_enabled", true);
}

void RCSignalBridge::loadConfig()
{
    // 加载 Topic 配置
    config_.rc_input_topic = this->get_parameter("rc_input_topic").as_string();
    config_.sport_state_topic = this->get_parameter("sport_state_topic").as_string();
    config_.low_state_topic = this->get_parameter("low_state_topic").as_string();
    config_.output_topic = this->get_parameter("output_topic").as_string();
    
    // 加载超时配置
    config_.rc_timeout_sec = this->get_parameter("rc_timeout_sec").as_double();
    config_.state_timeout_sec = this->get_parameter("state_timeout_sec").as_double();
    
    // 加载 QoS 配置
    config_.qos_depth = this->get_parameter("qos_depth").as_int();
    
    // 加载摇杆配置
    float deadzone = static_cast<float>(this->get_parameter("joystick_deadzone").as_double());
    bool deadzone_enabled = this->get_parameter("joystick_deadzone_enabled").as_bool();
    
    config_.left_stick_config.deadzone = deadzone;
    config_.left_stick_config.enable_deadzone = deadzone_enabled;
    config_.right_stick_config.deadzone = deadzone;
    config_.right_stick_config.enable_deadzone = deadzone_enabled;
    
    // 更新处理器配置
    left_stick_processor_->setConfig(config_.left_stick_config);
    right_stick_processor_->setConfig(config_.right_stick_config);
}

void RCSignalBridge::setupSubscribers()
{
    auto qos = rclcpp::QoS(config_.qos_depth);
    
    // 订阅遥控器信号
    rc_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
        config_.rc_input_topic,
        qos,
        std::bind(&RCSignalBridge::wirelessControllerCallback, this, std::placeholders::_1)
    );
    
    // 订阅运动状态
    sport_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        config_.sport_state_topic,
        qos,
        std::bind(&RCSignalBridge::sportModeStateCallback, this, std::placeholders::_1)
    );
    
    // 订阅底层状态
    low_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        config_.low_state_topic,
        qos,
        std::bind(&RCSignalBridge::lowStateCallback, this, std::placeholders::_1)
    );
}

void RCSignalBridge::setupPublisher()
{
    auto qos = rclcpp::QoS(config_.qos_depth);
    
    rc_pub_ = this->create_publisher<msg::NativeRC>(
        config_.output_topic,
        qos
    );
}

void RCSignalBridge::wirelessControllerCallback(
    const unitree_go::msg::WirelessController::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 更新遥控器连接状态
    last_rc_time_ = this->now();
    rc_connected_.store(true);
    
    // 处理摇杆数据
    auto left_stick = processJoystick(msg->lx, msg->ly, *left_stick_processor_);
    auto right_stick = processJoystick(msg->rx, msg->ry, *right_stick_processor_);
    
    // 处理按键数据
    auto buttons = processButtons(msg->keys);
    
    // 构建机器人状态
    auto robot_state = buildRobotState();
    
    // 构建并发布消息
    msg::NativeRC output_msg;
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = "b2_rc_frame";
    
    output_msg.left_stick = left_stick;
    output_msg.right_stick = right_stick;
    output_msg.buttons = buttons;
    output_msg.robot_state = robot_state;
    
    output_msg.seq = seq_.fetch_add(1);
    
    // 计算处理延迟
    auto end_time = std::chrono::high_resolution_clock::now();
    auto latency = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    output_msg.bridge_latency_us = static_cast<uint32_t>(latency.count());
    
    // 发布消息
    rc_pub_->publish(output_msg);
    
    // 更新统计
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.rc_msg_count++;
        stats_.published_count++;
        // 更新平均延迟（简单移动平均）
        stats_.avg_latency_us = stats_.avg_latency_us * 0.99 + latency.count() * 0.01;
    }
}

void RCSignalBridge::sportModeStateCallback(
    const unitree_go::msg::SportModeState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    cached_state_.sport_mode = msg->mode;
    cached_state_.gait_type = msg->gait_type;
    cached_state_.foot_raise_height = msg->foot_raise_height;
    cached_state_.body_height = msg->body_height;
    cached_state_.last_sport_update = this->now();
    
    {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.sport_msg_count++;
    }
}

void RCSignalBridge::lowStateCallback(
    const unitree_go::msg::LowState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 从 BMS 状态获取电池信息
    // 注意：具体字段名需要根据 unitree_go/msg/LowState 定义调整
    cached_state_.battery_soc = msg->bms_state.soc;
    // cached_state_.battery_voltage = msg->bms_state.voltage;  // 如果存在
    // cached_state_.battery_current = msg->bms_state.current;  // 如果存在
    cached_state_.last_low_update = this->now();
    
    {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.low_msg_count++;
    }
}

msg::JoystickState RCSignalBridge::processJoystick(
    float x, float y, JoystickProcessor& processor)
{
    auto data = processor.process(x, y);
    
    msg::JoystickState state;
    state.x = data.x;
    state.y = data.y;
    state.magnitude = data.magnitude;
    state.angle = data.angle;
    
    return state;
}

msg::ButtonState RCSignalBridge::processButtons(uint16_t raw_keys)
{
    key_parser_->parse(raw_keys);
    key_parser_->updateEvents();
    
    const auto& keys = key_parser_->getKeys();
    
    msg::ButtonState state;
    state.r1 = keys.bits.R1;
    state.l1 = keys.bits.L1;
    state.r2 = keys.bits.R2;
    state.l2 = keys.bits.L2;
    state.start = keys.bits.start;
    state.select = keys.bits.select;
    state.f1 = keys.bits.F1;
    state.f2 = keys.bits.F2;
    state.a = keys.bits.A;
    state.b = keys.bits.B;
    state.x = keys.bits.X;
    state.y = keys.bits.Y;
    state.up = keys.bits.up;
    state.down = keys.bits.down;
    state.left = keys.bits.left;
    state.right = keys.bits.right;
    state.raw_keys = raw_keys;
    
    return state;
}

msg::RobotState RCSignalBridge::buildRobotState()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    msg::RobotState state;
    
    // 运动状态
    state.sport_mode = cached_state_.sport_mode;
    state.gait_type = cached_state_.gait_type;
    state.foot_raise_height = cached_state_.foot_raise_height;
    state.body_height = cached_state_.body_height;
    
    // 电源状态
    state.battery_soc = cached_state_.battery_soc;
    state.battery_voltage = cached_state_.battery_voltage;
    state.battery_current = cached_state_.battery_current;
    
    // 遥控器连接状态
    auto time_since_rc = (this->now() - last_rc_time_).seconds();
    state.rc_connected = rc_connected_.load() && (time_since_rc < config_.rc_timeout_sec);
    state.rc_battery = 255;  // 无法获取，设为无效值
    
    // 时间戳
    state.last_rc_timestamp = last_rc_time_;
    state.last_state_timestamp = cached_state_.last_sport_update;
    
    return state;
}

RCSignalBridge::Statistics RCSignalBridge::getStatistics() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

}  // namespace b2_native_rc_interface

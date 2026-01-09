/**
 * @file rc_test_subscriber.cpp
 * @brief 测试订阅者节点
 * 
 * 用于调试和验证 NativeRC 消息的接收情况。
 * 
 * @author Your Name
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#include <rclcpp/rclcpp.hpp>
#include "b2_native_rc_interface/msg/native_rc.hpp"

class RCTestSubscriber : public rclcpp::Node
{
public:
    RCTestSubscriber()
        : Node("rc_test_subscriber")
    {
        // 声明参数
        this->declare_parameter<std::string>("topic", "/b2_native_rc_signal");
        this->declare_parameter<bool>("verbose", false);
        
        std::string topic = this->get_parameter("topic").as_string();
        verbose_ = this->get_parameter("verbose").as_bool();
        
        // 创建订阅者
        subscription_ = this->create_subscription<b2_native_rc_interface::msg::NativeRC>(
            topic,
            10,
            std::bind(&RCTestSubscriber::callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic.c_str());
    }

private:
    void callback(const b2_native_rc_interface::msg::NativeRC::SharedPtr msg)
    {
        msg_count_++;
        
        // 每10条消息打印一次摘要
        if (msg_count_ % 10 == 0 || verbose_) {
            printSummary(msg);
        }
        
        // 检测按键变化
        checkButtons(msg);
    }
    
    void printSummary(const b2_native_rc_interface::msg::NativeRC::SharedPtr& msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "[#%u] Left: (%.2f, %.2f) mag=%.2f | Right: (%.2f, %.2f) mag=%.2f | "
            "Mode: %d | Gait: %d | SOC: %d%% | Latency: %u us",
            msg->seq,
            msg->left_stick.x, msg->left_stick.y, msg->left_stick.magnitude,
            msg->right_stick.x, msg->right_stick.y, msg->right_stick.magnitude,
            msg->robot_state.sport_mode,
            msg->robot_state.gait_type,
            msg->robot_state.battery_soc,
            msg->bridge_latency_us
        );
    }
    
    void checkButtons(const b2_native_rc_interface::msg::NativeRC::SharedPtr& msg)
    {
        const auto& b = msg->buttons;
        
        // 检测按下的按键
        std::string pressed_keys;
        
        if (b.r1) pressed_keys += "R1 ";
        if (b.l1) pressed_keys += "L1 ";
        if (b.r2) pressed_keys += "R2 ";
        if (b.l2) pressed_keys += "L2 ";
        if (b.start) pressed_keys += "Start ";
        if (b.select) pressed_keys += "Select ";
        if (b.a) pressed_keys += "A ";
        if (b.b) pressed_keys += "B ";
        if (b.x) pressed_keys += "X ";
        if (b.y) pressed_keys += "Y ";
        if (b.up) pressed_keys += "Up ";
        if (b.down) pressed_keys += "Down ";
        if (b.left) pressed_keys += "Left ";
        if (b.right) pressed_keys += "Right ";
        if (b.f1) pressed_keys += "F1 ";
        if (b.f2) pressed_keys += "F2 ";
        
        if (!pressed_keys.empty() && pressed_keys != last_pressed_) {
            RCLCPP_INFO(this->get_logger(), "Buttons: [%s]", pressed_keys.c_str());
            last_pressed_ = pressed_keys;
        } else if (pressed_keys.empty() && !last_pressed_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Buttons: [Released]");
            last_pressed_ = "";
        }
        
        // 检测组合键
        if (b.l2 && b.a) {
            RCLCPP_WARN(this->get_logger(), "Combo detected: L2+A (Stand Lock)");
        }
        if (b.l2 && b.b) {
            RCLCPP_WARN(this->get_logger(), "Combo detected: L2+B (Damping/E-Stop)");
        }
    }
    
    rclcpp::Subscription<b2_native_rc_interface::msg::NativeRC>::SharedPtr subscription_;
    uint64_t msg_count_ = 0;
    bool verbose_ = false;
    std::string last_pressed_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RCTestSubscriber>();
    
    RCLCPP_INFO(node->get_logger(),
        "========================================\n"
        " RC Test Subscriber\n"
        "========================================\n"
        " Listening for NativeRC messages...\n"
        " Press Ctrl+C to exit.\n"
        "========================================");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

/**
 * @file rc_simulator_node.cpp
 * @brief 遥控器信号模拟器 - 用于无 B2 硬件的本地测试
 * 
 * 功能说明：
 * - 模拟发布 /wirelesscontroller 消息（WirelessController）
 * - 模拟发布 /sportmodestate 消息（SportModeState）
 * - 模拟发布 /lowstate 消息（LowState）
 * 
 * 用途：
 * - 在没有真实 B2 机器人时进行代码测试
 * - 验证消息处理逻辑和数据转换
 * - 调试和开发阶段使用
 * 
 * 注意：
 * ⚠️ 模拟数据可能与真实 B2 有细微差异
 * ⚠️ 仅用于开发测试，不能替代真实硬件验证
 * 
 * @author B2 Native RC Interface Team
 * @date 2026-01-09
 */

#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/low_state.hpp>

#include <cmath>
#include <random>

class RCSimulatorNode : public rclcpp::Node
{
public:
    RCSimulatorNode() : Node("rc_simulator_node"), 
                        seq_(0),
                        button_state_(0),
                        time_counter_(0.0)
    {
        // 声明参数
        this->declare_parameter("publish_rate", 100.0);  // 100Hz
        this->declare_parameter("simulation_mode", "auto");  // auto, static, random
        this->declare_parameter("wireless_topic", "/wirelesscontroller");
        this->declare_parameter("sport_mode_topic", "/sportmodestate");
        this->declare_parameter("low_state_topic", "/lowstate");
        
        // 获取参数
        double rate = this->get_parameter("publish_rate").as_double();
        sim_mode_ = this->get_parameter("simulation_mode").as_string();
        
        // 创建发布者
        wireless_pub_ = this->create_publisher<unitree_go::msg::WirelessController>(
            this->get_parameter("wireless_topic").as_string(), 10);
        
        sport_mode_pub_ = this->create_publisher<unitree_go::msg::SportModeState>(
            this->get_parameter("sport_mode_topic").as_string(), 10);
        
        low_state_pub_ = this->create_publisher<unitree_go::msg::LowState>(
            this->get_parameter("low_state_topic").as_string(), 10);
        
        // 创建定时器
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&RCSimulatorNode::timerCallback, this));
        
        // 初始化随机数生成器
        random_gen_ = std::mt19937(std::random_device{}());
        random_dist_ = std::uniform_real_distribution<float>(-1.0f, 1.0f);
        
        RCLCPP_INFO(this->get_logger(), 
            "🎮 RC Simulator Node started - Mode: %s, Rate: %.1f Hz", 
            sim_mode_.c_str(), rate);
        RCLCPP_INFO(this->get_logger(), "📡 Publishing simulated remote controller data...");
        
        printControls();
    }

private:
    void timerCallback()
    {
        // 发布遥控器数据
        auto wireless_msg = generateWirelessController();
        wireless_pub_->publish(wireless_msg);
        
        // 发布运动模式状态（每10次发布一次，10Hz）
        if (seq_ % 10 == 0) {
            auto sport_msg = generateSportModeState();
            sport_mode_pub_->publish(sport_msg);
            
            auto low_msg = generateLowState();
            low_state_pub_->publish(low_msg);
        }
        
        seq_++;
        time_counter_ += 0.01;  // 假设100Hz
        
        // 每5秒打印一次状态
        if (seq_ % 500 == 0) {
            printStatus(wireless_msg);
        }
    }
    
    unitree_go::msg::WirelessController generateWirelessController()
    {
        unitree_go::msg::WirelessController msg;
        
        if (sim_mode_ == "auto") {
            // 自动模式：生成周期性变化的数据
            float t = time_counter_;
            
            // 左摇杆：圆周运动
            msg.lx = 0.5f * std::cos(t);
            msg.ly = 0.5f * std::sin(t);
            
            // 右摇杆：正弦波
            msg.rx = 0.3f * std::sin(t * 2.0f);
            msg.ry = 0.3f * std::cos(t * 1.5f);
            
            // 按键：周期性按下
            button_state_ = 0;
            if (static_cast<int>(t) % 3 == 0) {
                button_state_ |= (1 << 4);  // R1 button
            }
            if (static_cast<int>(t) % 5 == 0) {
                button_state_ |= (1 << 5);  // L1 button
            }
            
        } else if (sim_mode_ == "random") {
            // 随机模式：完全随机数据
            msg.lx = random_dist_(random_gen_);
            msg.ly = random_dist_(random_gen_);
            msg.rx = random_dist_(random_gen_);
            msg.ry = random_dist_(random_gen_);
            
            // 随机按键
            if (seq_ % 50 == 0) {  // 每0.5秒可能改变按键状态
                button_state_ = std::uniform_int_distribution<uint16_t>(0, 0xFFFF)(random_gen_);
            }
            
        } else {
            // static 模式：固定数据
            msg.lx = 0.0f;
            msg.ly = 0.5f;  // 前进
            msg.rx = 0.0f;
            msg.ry = 0.0f;
            button_state_ = (1 << 4);  // R1 pressed
        }
        
        msg.keys = button_state_;
        
        return msg;
    }
    
    unitree_go::msg::SportModeState generateSportModeState()
    {
        unitree_go::msg::SportModeState msg;
        
        // 模拟运动模式参数
        msg.mode = 1;  // 假设 1 = 站立模式
        msg.gait_type = 1;  // 假设 1 = 慢走
        msg.foot_raise_height = 0.08f;  // 8cm
        msg.body_height = 0.0f;  // 默认高度
        msg.position[0] = 0.0f;
        msg.position[1] = 0.0f;
        msg.position[2] = 0.0f;
        
        // 速度 - 根据遥控器摇杆生成
        msg.velocity[0] = 0.5f * std::sin(time_counter_ * 0.5f);  // vx
        msg.velocity[1] = 0.0f;  // vy
        msg.velocity[2] = 0.3f * std::cos(time_counter_ * 0.3f);  // vyaw
        
        return msg;
    }
    
    unitree_go::msg::LowState generateLowState()
    {
        unitree_go::msg::LowState msg;
        
        // 模拟电池状态
        // 注意：实际的 BMS 结构可能不同，这里使用简化版本
        // msg.bms.soc = 75.0f;  // 75% 电量
        
        // 由于不确定实际的 BMS 字段结构，我们留空或使用默认值
        // 真实测试时需要根据实际消息结构调整
        
        return msg;
    }
    
    void printStatus(const unitree_go::msg::WirelessController& msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "📊 [Seq: %d] LX: %.2f, LY: %.2f | RX: %.2f, RY: %.2f | Keys: 0x%04X",
            seq_, msg.lx, msg.ly, msg.rx, msg.ry, msg.keys);
    }
    
    void printControls()
    {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║       遥控器模拟器 - 测试模式说明                   ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  simulation_mode 参数：                              ║");
        RCLCPP_INFO(this->get_logger(), "║    • auto   - 自动周期性变化（圆周+正弦波）          ║");
        RCLCPP_INFO(this->get_logger(), "║    • random - 完全随机数据                           ║");
        RCLCPP_INFO(this->get_logger(), "║    • static - 固定数据（前进+R1按下）                ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  修改模式：                                          ║");
        RCLCPP_INFO(this->get_logger(), "║    ros2 run b2_native_rc_interface rc_simulator \\   ║");
        RCLCPP_INFO(this->get_logger(), "║        --ros-args -p simulation_mode:=random         ║");
        RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "");
    }
    
    // 成员变量
    rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr wireless_pub_;
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr sport_mode_pub_;
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr low_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string sim_mode_;
    uint32_t seq_;
    uint16_t button_state_;
    double time_counter_;
    
    std::mt19937 random_gen_;
    std::uniform_real_distribution<float> random_dist_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RCSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

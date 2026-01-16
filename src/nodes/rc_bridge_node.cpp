/**
 * @file rc_bridge_node.cpp
 * @brief 遥控器信号桥接节点入口
 * 
 * 启动 RCSignalBridge 节点，监听 B2 遥控器信号并发布到 ROS2。
 * 
 * @author Your Name
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#include <rclcpp/rclcpp.hpp>
#include "b2_native_rc_interface/bridge/rc_signal_bridge.hpp"

int main(int argc, char* argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点选项（不自动声明参数，由代码控制）
    rclcpp::NodeOptions options;
    
    // 创建并运行节点
    auto node = std::make_shared<b2_native_rc_interface::RCSignalBridge>(options);
    
    RCLCPP_INFO(node->get_logger(), 
        "========================================\n"
        " B2 Native RC Signal Bridge\n"
        "========================================\n"
        " Listening for Unitree B2 remote controller signals...\n"
        " Output Topic: /b2_native_rc_signal\n"
        " Press Ctrl+C to exit.\n"
        "========================================");
    
    // 运行节点
    rclcpp::spin(node);
    
    // 清理
    rclcpp::shutdown();
    
    return 0;
}

#!/bin/bash
#
# 键盘模拟器演示脚本
#

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              🎮 键盘模拟器使用演示                          ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "这个脚本会帮你快速体验键盘模拟器的功能"
echo ""

# 检查环境
if ! ros2 pkg list | grep -q "b2_native_rc_interface"; then
    echo "❌ 错误：b2_native_rc_interface 包未找到"
    echo "   请先运行: source ~/ros2_ws/install/setup.bash"
    exit 1
fi

echo "✅ 环境检查通过"
echo ""

# 提供选择
echo "请选择启动方式："
echo ""
echo "【1】仅启动键盘模拟器"
echo "    - 适合：只想测试按键输入"
echo "    - 在另一个终端查看输出"
echo ""
echo "【2】启动键盘模拟器 + 桥接节点"
echo "    - 适合：测试完整数据流"
echo "    - 需要在另一个终端查看输出"
echo ""
echo "【3】启动完整测试环境（3个节点）"
echo "    - 适合：一键完整测试"
echo "    - 包含：模拟器 + 桥接 + 订阅者"
echo ""
echo "【4】查看详细使用指南"
echo ""

read -p "请输入选择 (1-4): " choice

case $choice in
    1)
        echo ""
        echo "🚀 启动键盘模拟器..."
        echo ""
        echo "💡 提示："
        echo "  - 按 W/A/S/D 控制左摇杆"
        echo "  - 按 I/J/K/L 控制右摇杆"
        echo "  - 按 1-8 模拟按键"
        echo "  - 按 H 查看帮助"
        echo "  - 按 ESC 退出"
        echo ""
        echo "在另一个终端运行以下命令查看输出："
        echo "  ros2 topic echo /wirelesscontroller"
        echo ""
        sleep 2
        ros2 run b2_native_rc_interface rc_keyboard_simulator
        ;;
    
    2)
        echo ""
        echo "🚀 启动键盘模拟器和桥接节点..."
        echo ""
        echo "在另一个终端运行以下命令查看处理后的数据："
        echo "  ros2 topic echo /b2_native_rc_signal"
        echo ""
        sleep 2
        
        # 后台启动桥接节点
        ros2 run b2_native_rc_interface rc_bridge_node &
        BRIDGE_PID=$!
        
        sleep 1
        echo "✅ 桥接节点已启动 (PID: $BRIDGE_PID)"
        echo ""
        
        # 前台运行键盘模拟器
        ros2 run b2_native_rc_interface rc_keyboard_simulator
        
        # 清理
        kill $BRIDGE_PID 2>/dev/null
        ;;
    
    3)
        echo ""
        echo "🚀 启动完整测试环境..."
        echo ""
        ros2 launch b2_native_rc_interface rc_bridge_with_test.launch.py &
        LAUNCH_PID=$!
        
        sleep 2
        echo "✅ 桥接和测试节点已启动"
        echo ""
        echo "现在启动键盘模拟器（在当前终端）..."
        echo ""
        
        ros2 run b2_native_rc_interface rc_keyboard_simulator
        
        # 清理
        kill $LAUNCH_PID 2>/dev/null
        ;;
    
    4)
        echo ""
        cat ~/ros2_ws/src/B2_ROS_Signal/docs/KEYBOARD_SIMULATOR_GUIDE.md | less
        ;;
    
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac

echo ""
echo "👋 测试完成！"

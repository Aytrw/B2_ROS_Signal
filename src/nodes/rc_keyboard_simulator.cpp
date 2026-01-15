/**
 * @file rc_keyboard_simulator.cpp
 * @brief 键盘控制的遥控器模拟器 - 实时交互测试
 * 
 * 键位说明：
 * 
 * 【左摇杆控制】
 *   W/S - 前进/后退 (Y轴)
 *   A/D - 左/右平移 (X轴)
 * 
 * 【右摇杆控制】
 *   I/K - 上/下 (Y轴)  
 *   J/L - 左/右 (X轴)
 * 
 * 【按键模拟】
 *   1-8 - R1, L1, R2, L2, START, SELECT, F1, F2
 *   Q/E - A, B
 *   Z/C - X, Y
 *   方向键 - UP, DOWN, LEFT, RIGHT
 * 
 * 【控制】
 *   R - 重置所有值
 *   ESC - 退出
 */

#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/low_state.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <iostream>
#include <iomanip>

class KeyboardSimulator : public rclcpp::Node
{
public:
    KeyboardSimulator() : Node("rc_keyboard_simulator"), seq_(0)
    {
        // 创建发布者
        wireless_pub_ = this->create_publisher<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10);
        sport_mode_pub_ = this->create_publisher<unitree_go::msg::SportModeState>(
            "/sportmodestate", 10);
        low_state_pub_ = this->create_publisher<unitree_go::msg::LowState>(
            "/lowstate", 10);
        
        // 初始化状态
        resetState();
        
        // 配置终端为非阻塞模式
        setupTerminal();
        
        // 创建定时器 - 10Hz 更新显示和发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardSimulator::update, this));
        
        printHelp();
        
        RCLCPP_INFO(this->get_logger(), "⌨️  键盘模拟器启动！按键进行控制...");
    }
    
    ~KeyboardSimulator()
    {
        restoreTerminal();
    }

private:
    void setupTerminal()
    {
        // 保存原始终端设置
        tcgetattr(STDIN_FILENO, &old_tio_);
        
        // 设置新的终端模式
        struct termios new_tio = old_tio_;
        new_tio.c_lflag &= ~(ICANON | ECHO);  // 禁用规范模式和回显
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
        
        // 设置非阻塞读取
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    
    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }
    
    void resetState()
    {
        lx_ = 0.0f;
        ly_ = 0.0f;
        rx_ = 0.0f;
        ry_ = 0.0f;
        keys_ = 0;
    }
    
    void update()
    {
        // 读取键盘输入
        char ch;
        while (read(STDIN_FILENO, &ch, 1) > 0) {
            handleKey(ch);
        }
        
        // 发布消息
        publishMessages();
        
        // 更新显示（每10次更新一次，1秒）
        if (seq_ % 10 == 0) {
            printStatus();
        }
        
        seq_++;
    }
    
    void handleKey(char key)
    {
        const float step = 0.1f;
        
        switch (key) {
            // 左摇杆
            case 'w': case 'W': ly_ = std::min(ly_ + step, 1.0f); break;
            case 's': case 'S': ly_ = std::max(ly_ - step, -1.0f); break;
            case 'a': case 'A': lx_ = std::max(lx_ - step, -1.0f); break;
            case 'd': case 'D': lx_ = std::min(lx_ + step, 1.0f); break;
            
            // 右摇杆
            case 'i': case 'I': ry_ = std::min(ry_ + step, 1.0f); break;
            case 'k': case 'K': ry_ = std::max(ry_ - step, -1.0f); break;
            case 'j': case 'J': rx_ = std::max(rx_ - step, -1.0f); break;
            case 'l': case 'L': rx_ = std::min(rx_ + step, 1.0f); break;
            
            // 按键
            case '1': toggleButton(4); break;  // R1
            case '2': toggleButton(5); break;  // L1
            case '3': toggleButton(6); break;  // R2
            case '4': toggleButton(7); break;  // L2
            case '5': toggleButton(11); break; // START
            case '6': toggleButton(10); break; // SELECT
            case '7': toggleButton(8); break;  // F1
            case '8': toggleButton(9); break;  // F2
            
            case 'q': case 'Q': toggleButton(0); break; // A
            case 'e': case 'E': toggleButton(1); break; // B
            case 'z': case 'Z': toggleButton(2); break; // X
            case 'c': case 'C': toggleButton(3); break; // Y
            
            // 方向键（需要读取转义序列）
            case '\033': { // ESC 序列开始
                char seq[2];
                if (read(STDIN_FILENO, &seq[0], 1) > 0 && seq[0] == '[') {
                    if (read(STDIN_FILENO, &seq[1], 1) > 0) {
                        switch (seq[1]) {
                            case 'A': toggleButton(12); break; // UP
                            case 'B': toggleButton(13); break; // DOWN
                            case 'D': toggleButton(14); break; // LEFT
                            case 'C': toggleButton(15); break; // RIGHT
                        }
                    }
                } else {
                    // 单独的 ESC 键 - 退出
                    RCLCPP_INFO(this->get_logger(), "👋 退出模拟器");
                    rclcpp::shutdown();
                }
                break;
            }
            
            // 控制命令
            case 'r': case 'R':
                resetState();
                RCLCPP_INFO(this->get_logger(), "🔄 重置所有值");
                break;
            
            case 'h': case 'H':
                printHelp();
                break;
        }
    }
    
    void toggleButton(int bit)
    {
        keys_ ^= (1 << bit);  // 切换位
    }
    
    void publishMessages()
    {
        // 发布遥控器数据
        auto wireless = unitree_go::msg::WirelessController();
        wireless.lx = lx_;
        wireless.ly = ly_;
        wireless.rx = rx_;
        wireless.ry = ry_;
        wireless.keys = keys_;
        wireless_pub_->publish(wireless);
        
        // 发布机器人状态（模拟）
        if (seq_ % 10 == 0) {
            auto sport = unitree_go::msg::SportModeState();
            sport.mode = 1;
            sport.gait_type = 1;
            sport.foot_raise_height = 0.08f;
            sport_mode_pub_->publish(sport);
            
            auto low = unitree_go::msg::LowState();
            low_state_pub_->publish(low);
        }
    }
    
    void printStatus()
    {
        // 清除当前行并回到行首
        std::cout << "\r\033[K";
        
        // 显示摇杆状态
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "🕹️  L:(" << std::setw(5) << lx_ << "," << std::setw(5) << ly_ << ") "
                  << "R:(" << std::setw(5) << rx_ << "," << std::setw(5) << ry_ << ") ";
        
        // 显示按下的按键
        std::cout << "🎮 [";
        bool first = true;
        const char* button_names[] = {"A", "B", "X", "Y", "R1", "L1", "R2", "L2", 
                                       "F1", "F2", "SEL", "STA", "↑", "↓", "←", "→"};
        for (int i = 0; i < 16; i++) {
            if (keys_ & (1 << i)) {
                if (!first) std::cout << " ";
                std::cout << button_names[i];
                first = false;
            }
        }
        if (first) std::cout << "---";
        std::cout << "]     ";
        
        std::cout << std::flush;
    }
    
    void printHelp()
    {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║          ⌨️  键盘控制遥控器模拟器                           ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
        std::cout << "║  【左摇杆】           【右摇杆】           【按键】          ║\n";
        std::cout << "║    W - 前进             I - 上              1 - R1           ║\n";
        std::cout << "║    S - 后退             K - 下              2 - L1           ║\n";
        std::cout << "║    A - 左移             J - 左              3 - R2           ║\n";
        std::cout << "║    D - 右移             L - 右              4 - L2           ║\n";
        std::cout << "║                                             5 - START        ║\n";
        std::cout << "║  【动作按键】                               6 - SELECT       ║\n";
        std::cout << "║    Q - A 按钮           【方向键】          7 - F1           ║\n";
        std::cout << "║    E - B 按钮           ↑/↓/←/→            8 - F2           ║\n";
        std::cout << "║    Z - X 按钮                                                ║\n";
        std::cout << "║    C - Y 按钮           【控制】                             ║\n";
        std::cout << "║                         R - 重置            H - 帮助         ║\n";
        std::cout << "║                         ESC - 退出                           ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n💡 提示：\n";
        std::cout << "  - 按键会切换按钮状态（再按一次取消）\n";
        std::cout << "  - 摇杆值会累积，按 R 重置为 0\n";
        std::cout << "  - 在另一个终端运行: ros2 topic echo /b2_native_rc_signal\n";
        std::cout << "\n";
    }
    
    // 成员变量
    rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr wireless_pub_;
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr sport_mode_pub_;
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr low_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    struct termios old_tio_;
    
    uint32_t seq_;
    float lx_, ly_, rx_, ry_;
    uint16_t keys_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<KeyboardSimulator>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
}

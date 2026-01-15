/**
 * @file key_parser.hpp
 * @brief 遥控器按键解析器 - 与 Unitree 官方例程兼容
 * 
 * 本文件实现了与 Unitree 官方 SDK 兼容的按键解析功能：
 * - xKeySwitchUnion: 官方按键位掩码联合体
 * - Button: 官方按钮状态类（含边沿检测）
 * - Gamepad: 官方遥控器封装类
 * 
 * 参考文档: https://support.unitree.com/home/zh/B2_developer/Get%20Remote%20Control%20Status
 * 官方例程源码: unitree_sdk2/example/wireless_controller
 * 
 * @author B2 Native RC Interface Team
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#ifndef B2_NATIVE_RC_INTERFACE__KEY_PARSER_HPP_
#define B2_NATIVE_RC_INTERFACE__KEY_PARSER_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace b2_native_rc_interface
{

// ============================================================================
// 按键位掩码联合体 (Key Switch Union)
// ============================================================================
/**
 * @brief 按键位掩码联合体 - 与官方 xKeySwitchUnion 完全一致
 * 
 * 使用方法：
 * @code
 * xKeySwitchUnion key;
 * key.value = joystick.keys();  // 将获取到的遥控器键值赋给联合体中的value变量
 * 
 * // 通过判断联合体中的 components.A 值，可以判断A键是否被按下
 * if ((int)key.components.A == 1) {
 *     std::cout << "The key A is pressed" << std::endl;
 * }
 * @endcode
 * 
 * 位布局 (Bit Layout):
 * - Bit 0-7:  R1, L1, start, select, R2, L2, F1, F2
 * - Bit 8-15: A, B, X, Y, up, right, down, left
 */
typedef union
{
    struct
    {
        // Byte 0 (Bit 0-7) - 肩键和功能键
        uint8_t R1     : 1;  ///< Bit 0: 右肩键 (Right Bumper)
        uint8_t L1     : 1;  ///< Bit 1: 左肩键 (Left Bumper)
        uint8_t start  : 1;  ///< Bit 2: Start键 (开始键)
        uint8_t select : 1;  ///< Bit 3: Select键 (选择键)
        uint8_t R2     : 1;  ///< Bit 4: 右扳机 (Right Trigger)
        uint8_t L2     : 1;  ///< Bit 5: 左扳机 (Left Trigger)
        uint8_t F1     : 1;  ///< Bit 6: F1功能键
        uint8_t F2     : 1;  ///< Bit 7: F2功能键
        
        // Byte 1 (Bit 8-15) - 动作键和方向键
        uint8_t A      : 1;  ///< Bit 8: A键 (通常为确认/执行)
        uint8_t B      : 1;  ///< Bit 9: B键 (通常为取消/返回)
        uint8_t X      : 1;  ///< Bit 10: X键
        uint8_t Y      : 1;  ///< Bit 11: Y键
        uint8_t up     : 1;  ///< Bit 12: 方向键上
        uint8_t right  : 1;  ///< Bit 13: 方向键右
        uint8_t down   : 1;  ///< Bit 14: 方向键下
        uint8_t left   : 1;  ///< Bit 15: 方向键左
    } components;  ///< 按键组件 - 官方命名为 components
    
    uint16_t value;  ///< 原始16位值
} xKeySwitchUnion;

// ============================================================================
// 按钮状态类 (Button Class)
// ============================================================================
/**
 * @brief 单个按钮状态类 - 与官方 Button 类完全一致
 * 
 * 提供按键的三种状态：
 * - pressed: 当前是否处于按下状态
 * - on_press: 是否恰好处于按下瞬间（上升沿）
 * - on_release: 是否恰好处于松开瞬间（下降沿）
 * 
 * 使用方法：
 * @code
 * Button A;
 * A.update(key.components.A);  // 每帧更新按键状态
 * 
 * if (A.on_press) {
 *     // A键刚刚被按下
 *     press_count++;
 * }
 * @endcode
 */
class Button
{
public:
    Button() {}
    
    /**
     * @brief 更新按键状态 - 官方实现
     * 
     * 边沿检测逻辑：
     * - on_press = 当前按下 && 上一次未按下 (上升沿)
     * - on_release = 当前未按下 && 上一次按下 (下降沿)
     * 
     * @param state 当前按键状态 (true=按下, false=释放)
     */
    void update(bool state)
    {
        // 官方实现逻辑
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }
    
    bool pressed = false;    ///< 当前是否按下
    bool on_press = false;   ///< 是否刚刚按下（上升沿）
    bool on_release = false; ///< 是否刚刚释放（下降沿）
};

// ============================================================================
// 按键解析器类 (Key Parser Class)
// ============================================================================
/**
 * @brief 按键解析器类
 * 
 * 提供以下功能：
 * 1. 位掩码解析为独立布尔值
 * 2. 按键边沿检测（on_press/on_release）
 * 3. 组合键检测
 * 
 * 此类是对官方 xKeySwitchUnion 和 Button 的封装，
 * 提供更便捷的 ROS2 接口。
 */
class KeyParser
{
public:
    KeyParser();
    ~KeyParser() = default;
    
    /**
     * @brief 解析按键位掩码
     * @param raw_keys 16位原始按键值（来自 wirelesscontroller.keys()）
     */
    void parse(uint16_t raw_keys);
    
    /**
     * @brief 获取解析后的按键联合体
     * @return xKeySwitchUnion 常引用
     */
    const xKeySwitchUnion& getKeys() const { return keys_; }
    
    /**
     * @brief 获取原始按键值
     * @return 16位原始值
     */
    uint16_t getRawKeys() const { return keys_.value; }
    
    // ========== 便捷访问器 (使用 components 字段) ==========
    bool isR1Pressed() const { return keys_.components.R1; }
    bool isL1Pressed() const { return keys_.components.L1; }
    bool isR2Pressed() const { return keys_.components.R2; }
    bool isL2Pressed() const { return keys_.components.L2; }
    bool isStartPressed() const { return keys_.components.start; }
    bool isSelectPressed() const { return keys_.components.select; }
    bool isF1Pressed() const { return keys_.components.F1; }
    bool isF2Pressed() const { return keys_.components.F2; }
    bool isAPressed() const { return keys_.components.A; }
    bool isBPressed() const { return keys_.components.B; }
    bool isXPressed() const { return keys_.components.X; }
    bool isYPressed() const { return keys_.components.Y; }
    bool isUpPressed() const { return keys_.components.up; }
    bool isDownPressed() const { return keys_.components.down; }
    bool isLeftPressed() const { return keys_.components.left; }
    bool isRightPressed() const { return keys_.components.right; }
    
    // ========== 组合键检测 ==========
    /**
     * @brief 检测站立锁定组合键 (L2 + A)
     */
    bool isStandLockCombo() const { return keys_.components.L2 && keys_.components.A; }
    
    /**
     * @brief 检测阻尼/急停组合键 (L2 + B)
     */
    bool isDampingCombo() const { return keys_.components.L2 && keys_.components.B; }
    
    /**
     * @brief 检测特殊动作组合键 (L2 + R2 + X)
     */
    bool isSpecialActionCombo() const { return keys_.components.L2 && keys_.components.R2 && keys_.components.X; }
    
    // ========== 边沿检测 (使用官方 Button 类) ==========
    /**
     * @brief 获取指定按键的 Button 对象
     * @param key_name 按键名称 ("A", "B", "L1", etc.)
     * @return Button 对象的常引用
     */
    const Button& getButton(const std::string& key_name) const;
    
    /**
     * @brief 更新所有按键的边沿检测状态（在 parse 后调用）
     */
    void updateButtons();
    
    // ========== 直接访问 Button 对象 ==========
    Button R1, L1, start, select, R2, L2, F1, F2;
    Button A, B, X, Y;
    Button up, down, left, right;

private:
    xKeySwitchUnion keys_;       ///< 当前按键状态
    xKeySwitchUnion prev_keys_;  ///< 上一帧按键状态
    
    // 按键名称到 Button 指针的映射
    std::unordered_map<std::string, Button*> button_map_;
    
    /**
     * @brief 初始化按键映射
     */
    void initButtonMap();
};

}  // namespace b2_native_rc_interface

#endif  // B2_NATIVE_RC_INTERFACE__KEY_PARSER_HPP_

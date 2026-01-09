/**
 * @file key_parser.hpp
 * @brief 遥控器按键位掩码解析器
 * 
 * 将Unitree遥控器的16位按键位掩码解析为独立的布尔值。
 * 支持组合键检测和按键事件（按下/释放）的边沿检测。
 * 
 * @author Your Name
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

/**
 * @brief 按键位掩码联合体
 * 
 * 将16位整数映射到各个按键位。
 * 位布局遵循 Unitree SDK2 的定义。
 */
union KeySwitchUnion
{
    struct
    {
        // Byte 0 (Bit 0-7)
        uint8_t R1     : 1;  ///< Bit 0: 右肩键
        uint8_t L1     : 1;  ///< Bit 1: 左肩键
        uint8_t start  : 1;  ///< Bit 2: Start键
        uint8_t select : 1;  ///< Bit 3: Select键
        uint8_t R2     : 1;  ///< Bit 4: 右扳机
        uint8_t L2     : 1;  ///< Bit 5: 左扳机
        uint8_t F1     : 1;  ///< Bit 6: F1功能键
        uint8_t F2     : 1;  ///< Bit 7: F2功能键
        
        // Byte 1 (Bit 8-15)
        uint8_t A      : 1;  ///< Bit 8: A键
        uint8_t B      : 1;  ///< Bit 9: B键
        uint8_t X      : 1;  ///< Bit 10: X键
        uint8_t Y      : 1;  ///< Bit 11: Y键
        uint8_t up     : 1;  ///< Bit 12: 方向键上
        uint8_t right  : 1;  ///< Bit 13: 方向键右
        uint8_t down   : 1;  ///< Bit 14: 方向键下
        uint8_t left   : 1;  ///< Bit 15: 方向键左
    } bits;
    
    uint16_t value;  ///< 原始16位值
};

/**
 * @brief 单个按键的状态（含边沿检测）
 */
struct ButtonEvent
{
    bool pressed;     ///< 当前是否按下
    bool on_press;    ///< 是否刚刚按下（上升沿）
    bool on_release;  ///< 是否刚刚释放（下降沿）
    
    /**
     * @brief 更新按键状态
     * @param current_state 当前按键状态
     */
    void update(bool current_state)
    {
        on_press = current_state && !pressed;
        on_release = !current_state && pressed;
        pressed = current_state;
    }
};

/**
 * @brief 按键解析器类
 * 
 * 提供以下功能：
 * 1. 位掩码解析为独立布尔值
 * 2. 按键边沿检测（on_press/on_release）
 * 3. 组合键检测
 */
class KeyParser
{
public:
    KeyParser();
    ~KeyParser() = default;
    
    /**
     * @brief 解析按键位掩码
     * @param raw_keys 16位原始按键值
     */
    void parse(uint16_t raw_keys);
    
    /**
     * @brief 获取解析后的按键联合体
     * @return KeySwitchUnion 引用
     */
    const KeySwitchUnion& getKeys() const { return keys_; }
    
    /**
     * @brief 获取原始按键值
     * @return 16位原始值
     */
    uint16_t getRawKeys() const { return keys_.value; }
    
    // ========== 便捷访问器 ==========
    bool isR1Pressed() const { return keys_.bits.R1; }
    bool isL1Pressed() const { return keys_.bits.L1; }
    bool isR2Pressed() const { return keys_.bits.R2; }
    bool isL2Pressed() const { return keys_.bits.L2; }
    bool isStartPressed() const { return keys_.bits.start; }
    bool isSelectPressed() const { return keys_.bits.select; }
    bool isF1Pressed() const { return keys_.bits.F1; }
    bool isF2Pressed() const { return keys_.bits.F2; }
    bool isAPressed() const { return keys_.bits.A; }
    bool isBPressed() const { return keys_.bits.B; }
    bool isXPressed() const { return keys_.bits.X; }
    bool isYPressed() const { return keys_.bits.Y; }
    bool isUpPressed() const { return keys_.bits.up; }
    bool isDownPressed() const { return keys_.bits.down; }
    bool isLeftPressed() const { return keys_.bits.left; }
    bool isRightPressed() const { return keys_.bits.right; }
    
    // ========== 组合键检测 ==========
    /**
     * @brief 检测站立锁定组合键 (L2 + A)
     */
    bool isStandLockCombo() const { return keys_.bits.L2 && keys_.bits.A; }
    
    /**
     * @brief 检测阻尼/急停组合键 (L2 + B)
     */
    bool isDampingCombo() const { return keys_.bits.L2 && keys_.bits.B; }
    
    /**
     * @brief 检测特殊动作组合键 (L2 + R2 + X)
     */
    bool isSpecialActionCombo() const { return keys_.bits.L2 && keys_.bits.R2 && keys_.bits.X; }
    
    // ========== 边沿检测 ==========
    /**
     * @brief 获取按键事件状态
     * @param key_name 按键名称 ("A", "B", "L1", etc.)
     * @return ButtonEvent 结构
     */
    ButtonEvent getButtonEvent(const std::string& key_name) const;
    
    /**
     * @brief 更新边沿检测状态（在parse后调用）
     */
    void updateEvents();

private:
    KeySwitchUnion keys_;
    KeySwitchUnion prev_keys_;
    
    // 按键事件映射
    std::unordered_map<std::string, ButtonEvent> events_;
    
    /**
     * @brief 初始化事件映射
     */
    void initEvents();
};

}  // namespace b2_native_rc_interface

#endif  // B2_NATIVE_RC_INTERFACE__KEY_PARSER_HPP_

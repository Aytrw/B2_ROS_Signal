/**
 * @file key_parser.cpp
 * @brief 按键解析器实现 - 与 Unitree 官方例程兼容
 * 
 * 实现了官方 xKeySwitchUnion 和 Button 类的功能。
 * 参考文档: https://support.unitree.com/home/zh/B2_developer/Get%20Remote%20Control%20Status
 * 
 * @author B2 Native RC Interface Team
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#include "b2_native_rc_interface/utils/key_parser.hpp"

namespace b2_native_rc_interface
{

// ============================================================================
// KeyParser 构造函数
// ============================================================================
KeyParser::KeyParser()
{
    // 初始化按键联合体为零
    keys_.value = 0;
    prev_keys_.value = 0;
    
    // 初始化按键名称到 Button 指针的映射
    initButtonMap();
}

// ============================================================================
// 初始化按键映射
// ============================================================================
void KeyParser::initButtonMap()
{
    // 肩键和扳机键
    button_map_["R1"] = &R1;
    button_map_["L1"] = &L1;
    button_map_["R2"] = &R2;
    button_map_["L2"] = &L2;
    
    // 功能键
    button_map_["start"] = &start;
    button_map_["select"] = &select;
    button_map_["F1"] = &F1;
    button_map_["F2"] = &F2;
    
    // 动作键 ABXY
    button_map_["A"] = &A;
    button_map_["B"] = &B;
    button_map_["X"] = &X;
    button_map_["Y"] = &Y;
    
    // 方向键
    button_map_["up"] = &up;
    button_map_["down"] = &down;
    button_map_["left"] = &left;
    button_map_["right"] = &right;
}

// ============================================================================
// 解析按键位掩码
// ============================================================================
void KeyParser::parse(uint16_t raw_keys)
{
    // 保存上一次状态（用于边沿检测）
    prev_keys_ = keys_;
    
    // 将获取到的遥控器键值赋给联合体中的 value 变量
    // 这与官方例程的用法完全一致:
    // key.value = joystick.keys();
    keys_.value = raw_keys;
}

// ============================================================================
// 更新所有按键的边沿检测状态
// ============================================================================
void KeyParser::updateButtons()
{
    // 按照官方 Gamepad::Update() 的方式更新各按键状态
    // 使用 components 字段访问各按键位
    
    // 更新肩键和扳机键
    R1.update(keys_.components.R1);
    L1.update(keys_.components.L1);
    R2.update(keys_.components.R2);
    L2.update(keys_.components.L2);
    
    // 更新功能键
    start.update(keys_.components.start);
    select.update(keys_.components.select);
    F1.update(keys_.components.F1);
    F2.update(keys_.components.F2);
    
    // 更新动作键 ABXY
    A.update(keys_.components.A);
    B.update(keys_.components.B);
    X.update(keys_.components.X);
    Y.update(keys_.components.Y);
    
    // 更新方向键
    up.update(keys_.components.up);
    down.update(keys_.components.down);
    left.update(keys_.components.left);
    right.update(keys_.components.right);
}

// ============================================================================
// 获取指定按键的 Button 对象
// ============================================================================
const Button& KeyParser::getButton(const std::string& key_name) const
{
    auto it = button_map_.find(key_name);
    if (it != button_map_.end()) {
        return *(it->second);
    }
    // 如果找不到，返回 A 键作为默认值（避免空引用）
    return A;
}

}  // namespace b2_native_rc_interface

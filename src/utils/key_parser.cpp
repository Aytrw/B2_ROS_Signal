/**
 * @file key_parser.cpp
 * @brief 按键解析器实现
 * 
 * @author Your Name
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#include "b2_native_rc_interface/utils/key_parser.hpp"

namespace b2_native_rc_interface
{

KeyParser::KeyParser()
{
    keys_.value = 0;
    prev_keys_.value = 0;
    initEvents();
}

void KeyParser::initEvents()
{
    // 初始化所有按键事件
    events_["R1"] = ButtonEvent{};
    events_["L1"] = ButtonEvent{};
    events_["R2"] = ButtonEvent{};
    events_["L2"] = ButtonEvent{};
    events_["start"] = ButtonEvent{};
    events_["select"] = ButtonEvent{};
    events_["F1"] = ButtonEvent{};
    events_["F2"] = ButtonEvent{};
    events_["A"] = ButtonEvent{};
    events_["B"] = ButtonEvent{};
    events_["X"] = ButtonEvent{};
    events_["Y"] = ButtonEvent{};
    events_["up"] = ButtonEvent{};
    events_["down"] = ButtonEvent{};
    events_["left"] = ButtonEvent{};
    events_["right"] = ButtonEvent{};
}

void KeyParser::parse(uint16_t raw_keys)
{
    // 保存上一次状态
    prev_keys_ = keys_;
    
    // 更新当前状态
    keys_.value = raw_keys;
}

void KeyParser::updateEvents()
{
    // 更新各按键的边沿检测状态
    events_["R1"].update(keys_.bits.R1);
    events_["L1"].update(keys_.bits.L1);
    events_["R2"].update(keys_.bits.R2);
    events_["L2"].update(keys_.bits.L2);
    events_["start"].update(keys_.bits.start);
    events_["select"].update(keys_.bits.select);
    events_["F1"].update(keys_.bits.F1);
    events_["F2"].update(keys_.bits.F2);
    events_["A"].update(keys_.bits.A);
    events_["B"].update(keys_.bits.B);
    events_["X"].update(keys_.bits.X);
    events_["Y"].update(keys_.bits.Y);
    events_["up"].update(keys_.bits.up);
    events_["down"].update(keys_.bits.down);
    events_["left"].update(keys_.bits.left);
    events_["right"].update(keys_.bits.right);
}

ButtonEvent KeyParser::getButtonEvent(const std::string& key_name) const
{
    auto it = events_.find(key_name);
    if (it != events_.end()) {
        return it->second;
    }
    return ButtonEvent{};  // 返回默认状态
}

}  // namespace b2_native_rc_interface

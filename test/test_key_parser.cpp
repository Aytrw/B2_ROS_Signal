/**
 * @file test_key_parser.cpp
 * @brief 按键解析器单元测试 - 验证与官方 xKeySwitchUnion 的兼容性
 * 
 * 测试内容:
 * 1. 按键位掩码解析 (xKeySwitchUnion.components)
 * 2. 边沿检测 (Button.on_press / on_release)
 * 3. 组合键检测
 * 
 * 参考文档: https://support.unitree.com/home/zh/B2_developer/Get%20Remote%20Control%20Status
 * 
 * @author B2 Native RC Interface Team
 * @date 2026-01-09
 * @copyright BSD-3-Clause
 */

#include <gtest/gtest.h>
#include "b2_native_rc_interface/utils/key_parser.hpp"

using namespace b2_native_rc_interface;

class KeyParserTest : public ::testing::Test
{
protected:
    KeyParser parser;
};

// ============================================================================
// 测试初始状态
// ============================================================================
TEST_F(KeyParserTest, InitialState)
{
    EXPECT_EQ(parser.getRawKeys(), 0);
    EXPECT_FALSE(parser.isAPressed());
    EXPECT_FALSE(parser.isL1Pressed());
}

// ============================================================================
// 测试单个按键 - 验证 components 字段解析
// ============================================================================
TEST_F(KeyParserTest, SingleButton)
{
    // A 键在 Bit 8 (官方文档定义)
    parser.parse(1 << 8);
    EXPECT_TRUE(parser.isAPressed());
    EXPECT_FALSE(parser.isBPressed());
    
    // 验证通过 getKeys().components 访问
    const auto& keys = parser.getKeys();
    EXPECT_TRUE(keys.components.A);
    EXPECT_FALSE(keys.components.B);
    
    // L1 键在 Bit 1
    parser.parse(1 << 1);
    EXPECT_TRUE(parser.isL1Pressed());
    EXPECT_FALSE(parser.isAPressed());
}

// ============================================================================
// 测试组合键
// ============================================================================
TEST_F(KeyParserTest, ComboKeys)
{
    // L2 (Bit 5) + A (Bit 8) = 站立解锁
    uint16_t stand_lock = (1 << 5) | (1 << 8);
    parser.parse(stand_lock);
    
    EXPECT_TRUE(parser.isStandLockCombo());
    EXPECT_FALSE(parser.isDampingCombo());
    
    // L2 (Bit 5) + B (Bit 9) = 阻尼模式
    uint16_t damping = (1 << 5) | (1 << 9);
    parser.parse(damping);
    
    EXPECT_FALSE(parser.isStandLockCombo());
    EXPECT_TRUE(parser.isDampingCombo());
}

// ============================================================================
// 测试边沿检测 - 验证官方 Button 类的 on_press/on_release
// ============================================================================
TEST_F(KeyParserTest, EdgeDetection)
{
    // 初始状态 - 无按键
    parser.parse(0);
    parser.updateButtons();
    
    // 使用官方 Button 类直接访问
    EXPECT_FALSE(parser.A.pressed);
    EXPECT_FALSE(parser.A.on_press);
    EXPECT_FALSE(parser.A.on_release);
    
    // 按下 A 键
    parser.parse(1 << 8);
    parser.updateButtons();
    
    // 验证上升沿 (on_press)
    EXPECT_TRUE(parser.A.pressed);
    EXPECT_TRUE(parser.A.on_press);    // 刚刚按下
    EXPECT_FALSE(parser.A.on_release);
    
    // 保持按下 - on_press 应该变为 false
    parser.parse(1 << 8);
    parser.updateButtons();
    
    EXPECT_TRUE(parser.A.pressed);
    EXPECT_FALSE(parser.A.on_press);   // 不再是刚按下
    EXPECT_FALSE(parser.A.on_release);
    
    // 释放 - 验证下降沿 (on_release)
    parser.parse(0);
    parser.updateButtons();
    
    EXPECT_FALSE(parser.A.pressed);
    EXPECT_FALSE(parser.A.on_press);
    EXPECT_TRUE(parser.A.on_release);  // 刚刚释放
    
    // 保持释放 - on_release 应该变为 false
    parser.parse(0);
    parser.updateButtons();
    
    EXPECT_FALSE(parser.A.pressed);
    EXPECT_FALSE(parser.A.on_press);
    EXPECT_FALSE(parser.A.on_release); // 不再是刚释放
}

// ============================================================================
// 测试 getButton() 方法
// ============================================================================
TEST_F(KeyParserTest, GetButtonByName)
{
    parser.parse(1 << 8);  // A 键
    parser.updateButtons();
    
    const Button& btn = parser.getButton("A");
    EXPECT_TRUE(btn.pressed);
    EXPECT_TRUE(btn.on_press);
}

// ============================================================================
// 测试所有按键位 - 验证位布局与官方一致
// ============================================================================
TEST_F(KeyParserTest, AllButtons)
{
    // 所有按键按下 (0xFFFF = 16个按键全部按下)
    parser.parse(0xFFFF);
    
    // 验证所有便捷访问器
    EXPECT_TRUE(parser.isR1Pressed());
    EXPECT_TRUE(parser.isL1Pressed());
    EXPECT_TRUE(parser.isStartPressed());
    EXPECT_TRUE(parser.isSelectPressed());
    EXPECT_TRUE(parser.isR2Pressed());
    EXPECT_TRUE(parser.isL2Pressed());
    EXPECT_TRUE(parser.isF1Pressed());
    EXPECT_TRUE(parser.isF2Pressed());
    EXPECT_TRUE(parser.isAPressed());
    EXPECT_TRUE(parser.isBPressed());
    EXPECT_TRUE(parser.isXPressed());
    EXPECT_TRUE(parser.isYPressed());
    EXPECT_TRUE(parser.isUpPressed());
    EXPECT_TRUE(parser.isRightPressed());
    EXPECT_TRUE(parser.isDownPressed());
    EXPECT_TRUE(parser.isLeftPressed());
    
    // 验证通过 components 直接访问
    const auto& keys = parser.getKeys();
    EXPECT_TRUE(keys.components.R1);
    EXPECT_TRUE(keys.components.L1);
    EXPECT_TRUE(keys.components.start);
    EXPECT_TRUE(keys.components.select);
    EXPECT_TRUE(keys.components.A);
    EXPECT_TRUE(keys.components.B);
    EXPECT_TRUE(keys.components.X);
    EXPECT_TRUE(keys.components.Y);
    EXPECT_TRUE(keys.components.up);
    EXPECT_TRUE(keys.components.down);
    EXPECT_TRUE(keys.components.left);
    EXPECT_TRUE(keys.components.right);
}

// ============================================================================
// 测试 xKeySwitchUnion 的官方用法
// ============================================================================
TEST_F(KeyParserTest, OfficialUsage)
{
    // 模拟官方例程的用法:
    // xKeySwitchUnion key;
    // key.value = joystick.keys();
    // if ((int)key.components.A == 1) { ... }
    
    xKeySwitchUnion key;
    key.value = (1 << 8);  // 模拟 A 键按下
    
    // 官方判断方式
    EXPECT_EQ((int)key.components.A, 1);
    EXPECT_EQ((int)key.components.B, 0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

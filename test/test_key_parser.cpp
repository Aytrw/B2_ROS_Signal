/**
 * @file test_key_parser.cpp
 * @brief 按键解析器单元测试
 * 
 * @author Your Name
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

// 测试初始状态
TEST_F(KeyParserTest, InitialState)
{
    EXPECT_EQ(parser.getRawKeys(), 0);
    EXPECT_FALSE(parser.isAPressed());
    EXPECT_FALSE(parser.isL1Pressed());
}

// 测试单个按键
TEST_F(KeyParserTest, SingleButton)
{
    // A 键在 Bit 8
    parser.parse(1 << 8);
    EXPECT_TRUE(parser.isAPressed());
    EXPECT_FALSE(parser.isBPressed());
    
    // L1 键在 Bit 1
    parser.parse(1 << 1);
    EXPECT_TRUE(parser.isL1Pressed());
    EXPECT_FALSE(parser.isAPressed());
}

// 测试组合键
TEST_F(KeyParserTest, ComboKeys)
{
    // L2 (Bit 5) + A (Bit 8)
    uint16_t stand_lock = (1 << 5) | (1 << 8);
    parser.parse(stand_lock);
    
    EXPECT_TRUE(parser.isStandLockCombo());
    EXPECT_FALSE(parser.isDampingCombo());
    
    // L2 (Bit 5) + B (Bit 9)
    uint16_t damping = (1 << 5) | (1 << 9);
    parser.parse(damping);
    
    EXPECT_FALSE(parser.isStandLockCombo());
    EXPECT_TRUE(parser.isDampingCombo());
}

// 测试边沿检测
TEST_F(KeyParserTest, EdgeDetection)
{
    // 初始状态 - 无按键
    parser.parse(0);
    parser.updateEvents();
    
    auto event = parser.getButtonEvent("A");
    EXPECT_FALSE(event.pressed);
    EXPECT_FALSE(event.on_press);
    EXPECT_FALSE(event.on_release);
    
    // 按下 A 键
    parser.parse(1 << 8);
    parser.updateEvents();
    
    event = parser.getButtonEvent("A");
    EXPECT_TRUE(event.pressed);
    EXPECT_TRUE(event.on_press);
    EXPECT_FALSE(event.on_release);
    
    // 保持按下
    parser.parse(1 << 8);
    parser.updateEvents();
    
    event = parser.getButtonEvent("A");
    EXPECT_TRUE(event.pressed);
    EXPECT_FALSE(event.on_press);  // 不再是刚按下
    EXPECT_FALSE(event.on_release);
    
    // 释放
    parser.parse(0);
    parser.updateEvents();
    
    event = parser.getButtonEvent("A");
    EXPECT_FALSE(event.pressed);
    EXPECT_FALSE(event.on_press);
    EXPECT_TRUE(event.on_release);
}

// 测试所有按键位
TEST_F(KeyParserTest, AllButtons)
{
    // 所有按键按下
    parser.parse(0xFFFF);
    
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
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

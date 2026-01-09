#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
B2 Native RC Bridge 完整启动文件

同时启动：
1. 遥控器信号桥接节点
2. 测试订阅者节点（用于调试）

Author: Your Name
Date: 2026-01-09
License: BSD-3-Clause
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""
    
    # 获取包路径
    pkg_share = get_package_share_directory('b2_native_rc_interface')
    
    # 默认配置文件路径
    default_config_file = os.path.join(pkg_share, 'config', 'rc_bridge_params.yaml')
    
    # ========== Launch 参数 ==========
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the configuration YAML file'
    )
    
    enable_test_subscriber_arg = DeclareLaunchArgument(
        'enable_test_subscriber',
        default_value='true',
        description='Enable test subscriber node for debugging'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output for test subscriber'
    )
    
    # ========== 节点定义 ==========
    
    # 主桥接节点
    rc_bridge_node = Node(
        package='b2_native_rc_interface',
        executable='rc_bridge_node',
        name='b2_native_rc_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )
    
    # 测试订阅者节点
    test_subscriber_node = Node(
        package='b2_native_rc_interface',
        executable='rc_test_subscriber',
        name='rc_test_subscriber',
        output='screen',
        parameters=[{
            'topic': '/b2_native_rc_signal',
            'verbose': LaunchConfiguration('verbose'),
        }],
        emulate_tty=True,
    )
    
    # ========== 返回 Launch 描述 ==========
    return LaunchDescription([
        # 参数声明
        config_file_arg,
        enable_test_subscriber_arg,
        verbose_arg,
        
        # 节点
        rc_bridge_node,
        test_subscriber_node,
    ])

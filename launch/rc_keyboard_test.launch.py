#!/usr/bin/env python3
"""
键盘模拟器 + 桥接节点 + 测试订阅者 一起启动
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('b2_native_rc_interface')
    config_file = os.path.join(pkg_share, 'config', 'rc_bridge_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Full path to rc_bridge parameter file'
        ),
        
        # 键盘模拟器（在前台运行，可以看到输入提示）
        Node(
            package='b2_native_rc_interface',
            executable='rc_keyboard_simulator',
            name='rc_keyboard_simulator',
            output='screen',
            prefix='xterm -e' if os.environ.get('DISPLAY') else '',
        ),
        
        # 信号桥接节点
        Node(
            package='b2_native_rc_interface',
            executable='rc_bridge_node',
            name='rc_bridge_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
        
        # 测试订阅者
        Node(
            package='b2_native_rc_interface',
            executable='rc_test_subscriber',
            name='rc_test_subscriber',
            output='screen',
        ),
    ])

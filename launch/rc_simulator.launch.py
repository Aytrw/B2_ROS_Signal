#!/usr/bin/env python3
"""
遥控器模拟器启动文件 - 用于本地测试

用法示例：
  # 自动模式（默认）
  ros2 launch b2_native_rc_interface rc_simulator.launch.py
  
  # 随机模式
  ros2 launch b2_native_rc_interface rc_simulator.launch.py simulation_mode:=random
  
  # 静态模式
  ros2 launch b2_native_rc_interface rc_simulator.launch.py simulation_mode:=static
  
  # 修改发布频率
  ros2 launch b2_native_rc_interface rc_simulator.launch.py publish_rate:=50.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='auto',
            description='Simulation mode: auto, random, or static'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='100.0',
            description='Publishing rate in Hz'
        ),
        
        # 模拟器节点
        Node(
            package='b2_native_rc_interface',
            executable='rc_simulator',
            name='rc_simulator',
            output='screen',
            parameters=[{
                'simulation_mode': LaunchConfiguration('simulation_mode'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }]
        ),
    ])

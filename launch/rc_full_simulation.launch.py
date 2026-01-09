#!/usr/bin/env python3
"""
完整模拟测试环境启动文件

启动组件：
  1. rc_simulator - 模拟遥控器信号
  2. rc_bridge_node - 信号桥接节点
  3. rc_test_subscriber - 测试订阅者

这是最完整的本地测试配置，可以验证整个数据流。

用法：
  ros2 launch b2_native_rc_interface rc_full_simulation.launch.py
  
  # 使用随机模式
  ros2 launch b2_native_rc_interface rc_full_simulation.launch.py simulation_mode:=random
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('b2_native_rc_interface')
    config_file = os.path.join(pkg_share, 'config', 'rc_bridge_params.yaml')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='auto',
            description='Simulation mode for rc_simulator: auto, random, or static'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Full path to rc_bridge parameter file'
        ),
        
        # 1. 模拟器节点 - 生成假的遥控器数据
        Node(
            package='b2_native_rc_interface',
            executable='rc_simulator',
            name='rc_simulator',
            output='screen',
            parameters=[{
                'simulation_mode': LaunchConfiguration('simulation_mode'),
                'publish_rate': 100.0,
            }],
            prefix='xterm -e' if os.environ.get('USE_XTERM', '0') == '1' else '',
        ),
        
        # 2. 信号桥接节点 - 处理和转换数据
        Node(
            package='b2_native_rc_interface',
            executable='rc_bridge_node',
            name='rc_bridge_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
        
        # 3. 测试订阅者 - 显示最终输出
        Node(
            package='b2_native_rc_interface',
            executable='rc_test_subscriber',
            name='rc_test_subscriber',
            output='screen',
            parameters=[{
                'topic_name': '/b2_native_rc_signal',
            }]
        ),
    ])

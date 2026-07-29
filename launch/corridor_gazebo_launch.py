"""
走廊导航系统 - Gazebo 仿真环境启动脚本

功能说明：
1. 启动 Gazebo 仿真环境（使用自定义走廊世界）
2. 生成 TurtleBot3 机器人模型

使用方法：
  # 步骤 1：设置环境变量
  export TURTLEBOT3_MODEL=burger
  
  # 步骤 2：启动 Gazebo 仿真环境
  ros2 launch my_first_agv corridor_gazebo_launch.py
  
  # 步骤 3：等待 Gazebo 完全加载（约 5-10 秒）
  
  # 步骤 4：在另一个终端启动导航系统
  ros2 launch my_first_agv corridor_nav_launch.py

注意：
  - 此脚本仅启动 Gazebo 仿真环境，不包含导航节点
  - 导航节点需要在另一个终端中单独启动
  - 这样可以避免同时启动导致系统卡顿
"""

import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    my_agv_dir = get_package_share_directory('my_first_agv')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # 参数文件
    corridor_world = PathJoinSubstitution([my_agv_dir, 'worlds', 'corridor.world'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # --- 0. 声明启动参数 ---
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        
        # --- 1. 启动 Gazebo 仿真环境 ---
        # 使用自定义的走廊世界文件
        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_init.so', 
                 '-s', 'libgazebo_ros_factory.so',
                 corridor_world],
            output='screen'),
        
        # --- 2. 生成 TurtleBot3 机器人模型 ---
        # 初始位置：(-5, 0) 走廊起点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '-8',
                'y_pose': '0.0',
                'z_pose': '0.00',
                
            }.items(),
        ),
    ])

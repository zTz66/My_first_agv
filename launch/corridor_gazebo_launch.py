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
        # 初始位置：(-4.5, 0) 走廊起点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                # 机器人出生位置必须在走廊范围内（x∈[-5,1]），
                # 否则 CanyonLayer 会把起点标记为致命代价导致无法规划
                'x_pose': '-4.5',
                'y_pose': '0.0',
                'z_pose': '0.00',
                
            }.items(),
        ),
    ])

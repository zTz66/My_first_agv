"""
走廊导航系统启动脚本（不含 Gazebo）

功能说明：
1. 启动地图服务器
2. 启动 AMCL 定位节点
3. 启动规划器服务器
4. 启动控制器服务器（使用走廊控制器）
5. 启动行为树导航器
6. 启动生命周期管理器
7. 启动 RViz2 可视化

使用方法：
  # 步骤 1：先启动 Gazebo（在另一个终端）
  export TURTLEBOT3_MODEL=burger
  ros2 launch my_first_agv corridor_gazebo_launch.py
  
  # 步骤 2：等待 Gazebo 完全加载后，再启动导航
  ros2 launch my_first_agv corridor_nav_launch.py

注意：
  - 此脚本不包含 Gazebo，需要单独启动
  - 这样可以避免同时启动导致卡顿
"""

import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    my_agv_dir = get_package_share_directory('my_first_agv')
    turtlebot3_nav_dir = get_package_share_directory('turtlebot3_navigation2')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    
    # 参数文件
    nav_params = PathJoinSubstitution([my_agv_dir, 'config', 'my_nav_params.yaml'])
    map_file = PathJoinSubstitution([my_agv_dir, 'maps', 'corridor_map.yaml'])
    
    # URDF 文件路径（使用 xacro 处理）
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_file = os.path.join(turtlebot3_description_dir, 'urdf', f'turtlebot3_{turtlebot3_model}.urdf')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 行为树文件完整路径（必须在 LaunchDescription 外部定义）
    corridor_bt_xml = PathJoinSubstitution([my_agv_dir, 'config', 'corridor_bt.xml'])
    
    return LaunchDescription([
        # ==========================================
        # 0. 声明启动参数
        # ==========================================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        
        # ==========================================
        # 1. Robot State Publisher（关键！发布 TF 变换）
        # 使用 xacro 命令处理 URDF 文件，解析 ${namespace} 变量
        # 延迟 5 秒启动，确保 Gazebo 时钟已就绪
        # ==========================================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': Command(['xacro ', urdf_file])
                    }]
                ),
            ]
        ),
        
        # ==========================================
        # 2. 地图服务器
        # ==========================================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file}]
        ),
        
        # ==========================================
        # 2. AMCL 定位
        # ==========================================
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav_params, {'use_sim_time': use_sim_time}]
        ),
        
        # ==========================================
        # 4. 规划器服务器
        # ==========================================
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav_params, {'use_sim_time': use_sim_time}]
        ),
        
        # ==========================================
        # 5. 控制器服务器（使用走廊控制器）
        # ==========================================
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav_params, {'use_sim_time': use_sim_time}]
        ),
        
        # ==========================================
        # 6. 行为树导航器
        # ==========================================
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav_params, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': corridor_bt_xml,
                'default_nav_through_poses_bt_xml': corridor_bt_xml
            }]
        ),
        
        # ==========================================
        # 7. 生命周期管理器
        # ==========================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': [
                            'map_server', 
                            'amcl',
                            'planner_server', 
                            'controller_server',
                            'bt_navigator'
                        ],
                        'manager_timeout': 60.0
                    }]
                ),
            ]
        ),
        
        # ==========================================
        # 8. RViz2 可视化
        # ==========================================
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', PathJoinSubstitution([
                        turtlebot3_nav_dir, 'rviz', 'tb3_navigation2.rviz'
                    ])],
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
            ]
        ),
    ])
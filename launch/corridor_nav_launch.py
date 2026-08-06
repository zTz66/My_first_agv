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
        # 注意：必须在 Gazebo 启动后再启动导航，确保 /clock 已发布
        # ==========================================
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
        # 2. 静态定位（Fake Localization，替代 AMCL）
        #    删掉 AMCL 后，map->odom 由静态变换发布。
        #    实测确认：odom 原点 = map 原点 (0,0)（Gazebo 里程计原点在世界原点，
        #    不在机器人出生点），所以 map->odom 必须用单位变换。
        #    若加 (-4.5,0) 平移会把机器人 map 位姿算成 -9（双重偏移），
        #    导致起点落在错误位置、路径无法跟随。
        # ==========================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
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
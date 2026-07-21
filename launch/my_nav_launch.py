import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    # 获取包路径
    my_agv_dir = get_package_share_directory('my_first_agv')
    turtlebot3_nav_dir = get_package_share_directory('turtlebot3_navigation2')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 参数文件
    nav_params = PathJoinSubstitution([my_agv_dir, 'config', 'my_nav_params.yaml'])
    default_map = PathJoinSubstitution([my_agv_dir, 'maps', 'turtlebot3_world.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # --- 0. 声明启动参数 ---
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),

        # --- 1. 启动 Gazebo 仿真环境 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # --- 2. 地图服务器 ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': default_map}],
        ),

        # --- 3. AMCL 定位 ---
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav_params, {'use_sim_time': use_sim_time}]
        ),

        # --- 4. 规划器 (重点！加载 Global Costmap，包含你的 CanyonLayer 插件) ---
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav_params, {'use_sim_time': use_sim_time}]
        ),

        # --- 5. 控制器 (局部避障控制器) ---
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav_params, {'use_sim_time': use_sim_time}]
        ),

        # --- 6. 行为树导航器 (管理导航任务) ---
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav_params, {'use_sim_time': use_sim_time}]
        ),

        # --- 7. 生命周期管理器 (管理所有导航节点) ---
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': True},
                                {'node_names': ['map_server', 'amcl', 
                                                'planner_server', 'controller_server',
                                                'bt_navigator']},
                                {'manager_timeout': 60.0}]
                ),
            ]
        ),
        
        # --- 8. RViz2 可视化 ---
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', PathJoinSubstitution([turtlebot3_nav_dir, 'rviz', 'tb3_navigation2.rviz'])],
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
            ]
        )
    ])
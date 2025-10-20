# ~/ros2_ws/src/turtlebot3_nav2_demo/launch/nav.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明地图路径参数（默认使用项目内的地图）
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('turtlebot3_nav2_demo'),
            'maps',
            'map.yaml'
        ]),
        description='Full path to map yaml file to load'
    )

    # 启动 Nav2 的 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('turtlebot3_nav2_demo'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    return LaunchDescription([
        map_file_arg,
        nav2_launch
    ])
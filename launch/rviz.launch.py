from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    rviz_config = Path(get_package_share_directory('turtlebot3_nav2_demo')) / 'rviz' / 'nav2_demo.rviz'
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)],
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])
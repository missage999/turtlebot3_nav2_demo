# launch/spawn_obstacle.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction  # 新增TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_nav2_demo')
    model_path = os.path.join(pkg_share, 'models', 'moving_obstacle', 'model.sdf')

    spawn_entity = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'moving_obstacle',
             '-file', model_path,
             '-x', '-2', '-y', '0.5', '-z', '0'],
        output='screen'
    )

    controller_node = Node(
        package='turtlebot3_nav2_demo',
        executable='moving_obstacle_controller.py',
        name='moving_obstacle_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        spawn_entity,
        # 延迟10秒启动控制器，确保模型完全加载
        TimerAction(period=10.0, actions=[controller_node])
    ])
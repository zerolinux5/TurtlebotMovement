from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb_movement',
            executable='tb_movement',
            name='joy_to_move'
        ),
        Node(
            package='tb_movement',
            executable='lidar_parser',
            name='lidar_to_obs'
        ),
        Node(
            package='tb_movement',
            executable='local_planner',
            name='local_planner'
        )
    ])
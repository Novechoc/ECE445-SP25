from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='gripper_node',
            name='gripper_controller',
            parameters=[{'motor_port': '/dev/ttyAMA1'}, {'motor_address': 1}, {'min_pos': -50000}, {'max_pos': 850000}],
        )
    ])
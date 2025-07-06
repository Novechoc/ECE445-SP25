from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    fsr_node = Node(
        package='fsr_sensor',
        executable='fsr_node',
        name='fsr_node',
        output='screen',
	parameters=[{'gpio_pin': 2}],
    )

    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gripper_control'),
                'launch',
                'gripper_launch.py'
            )
        )
    )

    return LaunchDescription([
        fsr_node,
        gripper_launch
    ])

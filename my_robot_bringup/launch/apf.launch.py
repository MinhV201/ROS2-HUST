from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():
    apf_conf = os.path.join(get_package_share_path('my_robot_description'), 'config', 'apf_conf.yaml')
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='apf_controller',
            name='apf_controller',
            parameters=[apf_conf],
            output='screen'
        )
    ])

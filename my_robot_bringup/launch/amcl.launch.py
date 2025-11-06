import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

amcl_path = os.path.join(get_package_share_path("my_robot_description"), 'config','amcl.yaml')
map_path = os.path.join(get_package_share_path("my_robot_description"),'map', 'map.yaml')

def generate_launch_description():
    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_path]
    )

    map_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output = "screen",
            parameters=[{"use_sim_time": True}, {"yaml_filename": map_path}],
        )
    
    nav2_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output = "screen",
        parameters=[{"use_sim_time": True}, {"autostart": True}, {"node_names": ["map_server", "amcl"]}],
        emulate_tty=True
    )
    
    return LaunchDescription([
        nav2_amcl,
        map_node,
        nav2_lifecycle,
    ])

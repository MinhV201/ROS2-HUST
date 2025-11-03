from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('my_robot_description'), 
                             'urdf', 'my_robot.urdf.xacro')
    config_path = os.path.join(get_package_share_path('my_robot_description'),
                               'rviz', 'config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(get_package_share_directory('my_robot_description'), 'world')]
    )

    #config_rviz = ParameterValue(Command(['xacro ', config_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output = 'both',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', config_path],
        parameters=[{'use_sim_time': True}]
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[('gz_args', [' -r -v 4 maze.sdf'])]
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', Command(['xacro ', urdf_path]),
            '-name', 'my_robot'
        ],
        output='screen'
    )



    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                    '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                    '/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            output='screen',
            remappings=[
                ('/model/diff_drive/odometry', '/odom'),
                ('/odom/tf', '/tf'),
                ('/lidar', '/scan'),
            ]
    )
    return LaunchDescription([
        ign_resource_path,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        gz_sim_launch,
        spawn_entity_node,
        bridge
    ])
"""Launch mocap, object pose offsets, and car odom publication."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch mocap tracking for the robot and all configured objects.

    Returns:
        LaunchDescription: Launch definition for object and car outputs.
    """
    launch_arguments = [
        DeclareLaunchArgument(
            'pose_topic',
            default_value='pose',
            description='ROS topic for car pose output'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='ROS topic for car odometry output'
        ),
    ]

    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    asset_name = LaunchConfiguration('asset_name')

    nodes = [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('open_dubs_mocap'), 'launch', 'vrpn_launch.py'
                ])
            ])
        ),
        Node(  # Block / ramp PoseOffset node
            package='open_dubs_mocap',
            executable='block_pose',
            name='pose_offset_node',
            namespace=namespace,
            output='screen',
            parameters=[config_file]
        ),
        Node(  # Car odometry publisher node
            package='open_dubs_mocap',
            executable='car_odom_publisher',
            name='odom_publisher',
            namespace=namespace,
            output='screen',
            parameters=[config_file],
            remappings=[
                ('mocap_output_pose', 'vrpn_output_pose'),
                ('car_pose',  [asset_name, '/', LaunchConfiguration('pose_topic')]),
                ('car_odom',  [asset_name, '/', LaunchConfiguration('odom_topic')]),
            ]
        ),
    ]

    return LaunchDescription(launch_arguments + nodes)

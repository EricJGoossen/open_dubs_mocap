from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument(
            'pose_topic',
            default_value='pose',
            description='ROS topic for car pose output'
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
        Node(  # Car pose publisher node
            package='open_dubs_mocap',
            executable='car_pose_publisher',
            name='pose_publisher',
            namespace=namespace,
            output='screen',
            parameters=[config_file],
            remappings=[
                ('input_pose', ['vprn_mocap/', asset_name, '/pose']),
                ('output_pose', [asset_name, '/', LaunchConfiguration('pose_topic')]),
            ]
        ),
    ]

    return LaunchDescription(launch_arguments + nodes)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_path = PathJoinSubstitution([
        FindPackageShare('open_dubs_mocap'), 'config', 'mocap_defaults.yaml'
    ])

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
        DeclareLaunchArgument(
            'namespace',
            default_value='mocap',
            description='Namespace for all launched nodes'
        ),
        DeclareLaunchArgument(
            'asset_name',
            default_value='opendubs',
            description='Name of the asset being tracked (used for topic remapping)'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path,
            description='Path to the YAML configuration file for the mocap nodes'
        )
    ]

    launches = [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('open_dubs_mocap'), 'launch', 'vrpn_launch.py'
                ])
            ])
        )
    ]

    asset_name = LaunchConfiguration('asset_name')
    nodes = [
        Node(  # Car odometry publisher node
            package='open_dubs_mocap',
            executable='car_odom_publisher',
            name='odom_publisher',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('mocap_output_pose', 'vrpn_output_pose'),
                ('car_pose',  [asset_name, '/', LaunchConfiguration('pose_topic')]),
                ('car_odom',  [asset_name, '/', LaunchConfiguration('odom_topic')]),
            ]
        ),
    ]

    return LaunchDescription(launch_arguments + launches + nodes)

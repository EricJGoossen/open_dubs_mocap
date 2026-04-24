from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
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
        ),
        DeclareLaunchArgument(
            'server',
            default_value='192.168.0.184',
            description='VRPN server IP'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='3883',
            description='VRPN server port'
        ),
        DeclareLaunchArgument(
            'use_fake_mocap',
            default_value='false',
            description='Use fake mocap instead of VRPN'
        )
    ]

    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    asset_name = LaunchConfiguration('asset_name')
    vrpn_name = 'vrpn_mocap'

    launches = [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(vrpn_name), 'launch', 'client.launch.yaml'
                ])
            ]),
            condition=UnlessCondition(LaunchConfiguration('use_fake_mocap')),
            launch_arguments={
                'server': LaunchConfiguration('server'),
                'port': LaunchConfiguration('port'),
            }.items()
        ),
    ]
    nodes = [
        Node(  # Fake mocap node
            package='open_dubs_mocap',
            executable='fake_mocap',
            name='fake_mocap',
            namespace=namespace,
            output='screen',
            remappings=[
                ('car_pose', [vrpn_name, '/', asset_name, '/pose']),
                ('ramp1_pose', vrpn_name + '/ramp1/pose'),
                ('ramp2_pose', vrpn_name + '/ramp2/pose'),
                ('block1_pose', vrpn_name + '/block1/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('use_fake_mocap'))
        ),
        Node(  # Mocap relay node
            package='open_dubs_mocap',
            executable='relay_mocap',
            name='relay',
            namespace=namespace,
            output='screen',
            parameters=[config_file],
            remappings=[
                ('input_pose', ['/' + vrpn_name + '/', asset_name, '/pose']),
                ('output_pose', 'vrpn_output_pose'),
            ],
        ),
        Node(  # Car odometry publisher node
            package='open_dubs_mocap',
            executable='car_odom_publisher',
            name='odom_publisher',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[config_file],
            remappings=[
                ('mocap_output_pose', 'vrpn_output_pose'),
                ('car_pose',  [asset_name, '/', LaunchConfiguration('pose_topic')]),
                ('car_odom',  [asset_name, '/', LaunchConfiguration('odom_topic')]),
            ]
        ),
    ]

    return LaunchDescription(launch_arguments + launches + nodes)

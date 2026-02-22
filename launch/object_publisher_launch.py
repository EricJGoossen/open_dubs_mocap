from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_config_file = PathJoinSubstitution([
        FindPackageShare("open_dubs_mocap"), "config", "mocap_defaults.yaml"
    ])

    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            "config_file", 
            default_value=default_config_file,
            description="Path to YAML config file for mocap nodes"
        ),
        DeclareLaunchArgument(
            'use_fake_mocap',
            default_value='false',
            description='Use fake mocap instead of VRPN'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='mocap',
            description='ROS namespace for mocap topics'
        ),
        DeclareLaunchArgument(
            'asset_name',
            default_value='open_dubs',
            description='Name of the asset being tracked by the mocap'
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='pose',
            description='ROS topic for car pose output'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='ROS topic for car odometry output'
        )
    ]

    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    asset_name = LaunchConfiguration('asset_name')
    vrpn_name = 'vrpn_client_node'

    nodes = [
        Node( # VRPN client node
            package='vrpn_client_ros',
            executable='vrpn_client_node',
            name=vrpn_name,
            namespace=namespace,
            output='screen',
            parameters=[
                config_file,
                {'asset_name': asset_name}
            ],
            condition=UnlessCondition(LaunchConfiguration("use_fake_mocap")),
        ),
        Node( # Fake mocap node
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
        Node( # Block / ramp PoseOffset node
            package='open_dubs_mocap',
            executable='block_pose',
            name='pose_offset_node',
            namespace=namespace,
            output='screen',
            parameters=[config_file]
        ),
        Node( # Mocap relay node
            package='open_dubs_mocap',
            executable='relay_mocap',
            name='relay',
            namespace=namespace,
            output='screen',
            remappings=[
                ('input_pose', [vrpn_name + '/', asset_name, '/pose']),
                ('output_pose', 'vrpn_output_pose'),
            ],
        ),
        Node( # Car odometry publisher node
            package='open_dubs_mocap',
            executable='car_odom_publisher',
            name='odom_publisher',
            namespace=namespace,
            output='screen',
            parameters=[config_file],
            remappings=[
                ('mocap_output_pose', 'vrpn_output_pose'),
                ('car_pose', [asset_name, '/', LaunchConfiguration('pose_topic')]),
                ('car_odom', [asset_name, '/', LaunchConfiguration('odom_topic')])
            ]
        )
    ]   
    return LaunchDescription(launch_arguments + nodes)

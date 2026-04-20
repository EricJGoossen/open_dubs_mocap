from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_config_path = PathJoinSubstitution([
        FindPackageShare('open_dubs_mocap'), 'config', 'mocap_defaults.yaml'
    ])

    launch_args = [
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
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('open_dubs_mocap'), 'launch', 'odom_publisher_launch.py'
                ])
            ]),
            launch_arguments={
                'pose_topic': 'mocap_output_pose',
                'odom_topic': 'mocap_output_odom',
                'namespace': LaunchConfiguration('namespace'),
                'asset_name': LaunchConfiguration('asset_name'),
                'config_file': LaunchConfiguration('config_file')
            }.items()
        ),
    ]

    nodes = [
        Node(
            package='open_dubs_mocap',
            executable='odom_broadcaster',
            name='odom_tf_broadcaster',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[('odom_input', [LaunchConfiguration('asset_name'), '/', 'mocap_output_pose'])]
        )
    ]

    return LaunchDescription(launch_args + launches + nodes)
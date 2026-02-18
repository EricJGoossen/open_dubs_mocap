#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('open_dubs_mocap')
    config_file = os.path.join(pkg_share, 'config', 'mocap_defaults.yaml')

    # Load configuration parameters from YAML file
    config_data = {}
    try:
        with open(config_file, 'r', encoding='utf-8') as config_stream:
            config_data = yaml.safe_load(config_stream) or {}
    except FileNotFoundError:
        config_data = {}

    # Extract ROS parameters
    ros_params = config_data.get('/**', {}).get('ros__parameters', {})
    asset_name = ros_params.get('car_name', None)

    tracked_objects = ros_params.get('tracked_objects', [])
    if not isinstance(tracked_objects, list):
        tracked_objects = []

    trackers = ros_params.get('trackers', [])
    if asset_name or tracked_objects:
        trackers = ([] if asset_name is None else [asset_name]) + tracked_objects

    # Declare use_fake_mocap launch argumen
    # Declare launch arguments
    use_fake_mocap_arg = DeclareLaunchArgument(
        'use_fake_mocap',
        default_value='false',
        description='Use fake mocap instead of VRPN'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='mocap',
        description='ROS namespace for mocap topics'
    )

    use_fake = LaunchConfiguration('use_fake_mocap')
    namespace = LaunchConfiguration('namespace')

    # VRPN client node
    vrpn_node = Node(
        package='vrpn_client_ros', 
        executable='vrpn_client_node',
        name='vrpn_client_node',
        output='screen',
        parameters=[
            config_file,
            {'trackers': trackers}
        ],
        condition=IfCondition(PythonExpression(["'", use_fake, "' == 'false'"]))
    )

    # Fake mocap node
    fake_mocap_node = Node(
        package='open_dubs_mocap',
        executable='fake_mocap',
        name='fake_mocap',
        output='screen',
        remappings=[
            ('car_pose', f'/vrpn_client_node/{asset_name}/pose'),
            ('ramp1_pose', '/vrpn_client_node/ramp1/pose'),
            ('ramp2_pose', '/vrpn_client_node/ramp2/pose'),
            ('block1_pose', '/vrpn_client_node/block1/pose'),
        ],
        condition=IfCondition(use_fake)
    )

    # Mocap relay node
    mocap_relay_node = Node(
        package='open_dubs_mocap',
        executable='relay_mocap',
        name='relay',
        namespace=namespace,
        output='screen',
        remappings=[
            ('input_pose', f'/vrpn_client_node/{asset_name}/pose'),
            ('output_pose', 'mocap_pose'),
        ],
    )

    return LaunchDescription([
        use_fake_mocap_arg,
        namespace_arg,
        vrpn_node,
        fake_mocap_node,
        mocap_relay_node
    ])

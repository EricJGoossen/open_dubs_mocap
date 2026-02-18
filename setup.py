from setuptools import find_packages, setup

package_name = 'open_dubs_mocap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/car_odom_publisher_launch.py',
            'launch/car_pose_publisher_launch.py',
            'launch/object_publisher_launch.py',
            'launch/vrpn_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/mocap_tf_offset.yaml',
            'config/mocap_defaults.yaml'
        ]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Eric Goossen',
    maintainer_email='fada@uw.edu',
    description='MoCap utilities and odometry publishers ported to ROS 2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
         'console_scripts': [
            'car_pose_publisher = open_dubs_mocap.car_pose_publisher:main',
            'car_odom_publisher = open_dubs_mocap.car_odom_publisher:main',
            'fake_mocap = open_dubs_mocap.fake_mocap:main',
            'relay_mocap = open_dubs_mocap.relay_mocap:main',
            'block_pose = open_dubs_mocap.block_pose:main',
        ],
    },
)

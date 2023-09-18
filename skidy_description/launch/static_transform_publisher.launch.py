import os
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    ld = LaunchDescription()
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mapToOdom',
        output='log',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        respawn=True,
        respawn_delay=3
        )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camTolink',
        output='log',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1',
            '--frame-id', 'camera_bottom_screw_frame',
            '--child-frame-id', 'device/base_link/realsense_d435'
         ],
        respawn=True,
        respawn_delay=3
         )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imuTolink',
        output='log',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1',
            '--frame-id', 'imu_link',
            '--child-frame-id', 'device/base_link/imu'
         ],
        respawn=True,
        respawn_delay=3
        )

    navsat_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='navsatTolink',
        output='log',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1',
            '--frame-id', 'navsat_link',
            '--child-frame-id', 'device/base_link/navsat'
        ],
        respawn=True,
        respawn_delay=3
        )

    ld.add_action(map_odom_tf)
    ld.add_action(camera_tf)
    ld.add_action(imu_tf)
    ld.add_action(navsat_tf)

    return ld


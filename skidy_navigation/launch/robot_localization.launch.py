import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_localization_dir = get_package_share_directory('skidy_navigation')
    parameters_file_dir = os.path.join(robot_localization_dir, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'localization.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    output_final_position = DeclareLaunchArgument(
                            'output_final_position',
                            default_value='false')

    output_location = DeclareLaunchArgument(
                        'output_location',
                        default_value='~/dual_ekf_navsat_example_debug.txt')

    ekf_filter_node_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[parameters_file_path],
        # remappings=[('odometry/filtered', 'odometry/local')]
    )

    ekf_filter_node_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[parameters_file_path],
        # remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='log',  # or screen
        parameters=[parameters_file_path],
        remappings=[
            # # # Subscribed topics # # #
            ('imu/data', '/device/imu/data'),
            ('gps/fix', '/navsat/fix'),
            # ('odometry/filtered', 'odometry/global'),
            # # # Published topics # # #
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
                    ],
        respawn=True,
        respawn_delay=4,
    )

    ld = LaunchDescription()

    ld.add_action(output_final_position)
    ld.add_action(output_location)
    # ld.add_action(ekf_filter_node_map)
    ld.add_action(ekf_filter_node_odom)
    ld.add_action(navsat_transform)

    return ld



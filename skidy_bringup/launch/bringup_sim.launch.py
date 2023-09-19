import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    nav_pkg_name = "skidy_navigation"
    description_pkg_name = "skidy_description"
    rviz_file_name = 'test.rviz'
    urdf_file_name = "robot_test_diff_drive.urdf"
    world_name = 'primitives.sdf'  
    world_path = os.path.join(get_package_share_directory(description_pkg_name), 'worlds', world_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    urdf_file = str(get_package_share_path(description_pkg_name) / 'urdf' / urdf_file_name)
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str)

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(get_package_share_directory(description_pkg_name), 'rviz', rviz_file_name),
        description='Full path to the RVIZ config file to use')

    # Robot state publisher
    # params = {'use_sim_time': True, 'robot_description': LaunchConfiguration('robot_description')}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        # arguments=[]
    )

    # RViz
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        # launch_arguments={'gz_args':  '-r empty.sdf'}.items(),
        launch_arguments={'gz_args':  world_path}.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name="gazebo_spawner",
        arguments=[
            '-name', 'device',
            '-x', '0.0',
            '-z', '2.5',
            '-y', '0.0',
            # '-file', urdf_file,
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/device/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                   '/navsat/fix@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
                   '/rgbd/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/rgbd/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                   '/rgbd/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                   '/rgbd/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/odom@nav_msgs/msg/Odometry@ignition.msgs.OdometryWithCovariance',
                   '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
                   ],
        output='screen'
    )

    static_transforms = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('skidy_description'), 'launch',
                     'static_transform_publisher.launch.py')
    )

    rtabmap_odometry = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('skidy_navigation'), 'launch',
                     'rtabmap_odometry.launch.py')
    )

    robot_localization = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('skidy_navigation'), 'launch',
                     'robot_localization.launch.py')
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher"
    )
   
    ld = LaunchDescription()

    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(static_transforms)
    ld.add_action(gazebo)
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn)
    ld.add_action(bridge)
    # ld.add_action(rtabmap_odometry)
    # ld.add_action(robot_localization)
    ld.add_action(rviz)

    return ld
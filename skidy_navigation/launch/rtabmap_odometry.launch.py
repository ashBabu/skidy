import os
from launch import LaunchDescription
from launch_ros.actions import SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    rtabmap_ros = GroupAction(
        actions=[
            SetRemap(src='/device/imu/data', dst='/rtabmap/device/imu/data'),
            SetRemap(src='/navsat/fix', dst='/rtabmap/navsat/fix'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
                launch_arguments={
                    "rtabmap_args": "--delete_db_on_start",
                    "rgb_topic": "/rgbd/camera/image",
                    "depth_topic": "/rgbd/camera/depth_image",
                    "camera_info_topic": "/rgbd/camera/camera_info",
                    "frame_id": "base_link",
                    "approx_sync": "true",
                    "approx_sync_max_interval": "0.01",
                    "wait_imu_to_init": "true",
                    "imu_topic": "/device/imu/data",
                    "qos": "2",
                    "rviz": "false",
                    "publish_tf": "true",
                    "use_sim_time": "true",
                    "visual_odometry": "true",
                    "gps_topic": "/navsat/fix",
                    "rtabmapviz": "false",
                    "queue_size": "200",
                    "publish_tf_map": "false",
                    "rgbd_sync": "true",
                    "output": "log",
                    # # # (uncomment the following if visual odometry is false) # # #
                    # "icp_odometry": "true",
                    # "scan_cloud": "/rgbd/camera/points",
                    # "scan_downsampling_step": "2",
                    # "scan_voxel_size": "0.03"
                }.items()
            )
        ]
    )

    ld.add_action(rtabmap_ros)

    return ld

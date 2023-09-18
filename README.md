
### Requirements
* Ubuntu 22.04, ROS2 Humble, Gazebo Fortress
* `sudo apt install ros-${ROS_DISTRO}-realsense2-camera &&
  ros-${ROS_DISTRO}-realsense2-description &&
  ros-${ROS_DISTRO}-rtabmap-ros &&
  ros-${ROS_DISTRO}-imu-tools &&
  ros-${ROS_DISTRO}-rviz2 &&
  ros-${ROS_DISTRO}-rqt &&
  ros-${ROS_DISTRO}-tf2-tools &&
  ros-${ROS_DISTRO}-ros-gz &&
  ros-dev-tools`

* In your `~/.bashrc`, add the following

  `source /opt/ros/humble/setup.bash`

   `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`

   `source /usr/share/colcon_cd/function/colcon_cd.sh`

### Step by step instructions to ROS2 Navigation stack
**Setup your colcon workspace**

* `mkdir -p ~/skidy_ws/src`
* `cd skidy_ws/src`
* `git clone https://github.com/ashBabu/skidy.git`
* `git fetch origin --prune`
* `cd skidy`
* `git checkout ros2-ignition-gazebo`
* `cd ~/skidy_ws`
* `colcon build --symlink-install`
* `source install/setup.bash`

### Differences that I found in Gazebo Fortress compared to classic gazebo (gazebo_ros)
* Topics published can only be viewed by the command `ign topic -l`. The subscribed topics remain [hidden](https://github.com/gazebosim/gz-sim/issues/1672#issuecomment-1341296940)
* All topics published or subscribed by Fortress requires a [ros_ign_bridge](http://wiki.ros.org/ros_ign_bridge) to be available in ROS as in [here](https://github.com/ashBabu/skidy/blob/ros2-ignition-gazebo/skidy_bringup/launch/bringup_sim.launch.py#L83)
* The `/clock` is published by Fortress but not available in ROS by default even though there is `/clock` displayed by the command `ros2 topic list`.
  For this to be available, the bridge needs to be [properly set up](https://github.com/ashBabu/skidy/blob/ros2-ignition-gazebo/skidy_bringup/launch/bringup_sim.launch.py#L96). This
  is a unidirectional message from Fortress to ROS provided by the symbol `[`. More details [here](https://github.com/gazebosim/gz-sim/issues/1361#issuecomment-1050069078)
* For other msg types, set up the brige as required. Documentation available [here](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)
* Echo command in Fortress is `ign topic -e -t /topic_name`
* Although Fortress is the [recommended version](https://gazebosim.org/docs/harmonic/ros_installation#summary-of-compatible-ros-and-gazebo-combinations) of ROS2 Humble, the namespace
  `ignition` has already been replaced by `gz`

### Launch
* `ros2 launch skidy_bringup bringup_sim.launch.py` 
* Press the play button in Gazebo fortress

### To publish to cmd_vel topic
* `ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.5}"`
  

                                    OR


* `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"`

**Notes**
* The topic `cmd_vel` or something that you prefer has to be set specifically in the plugin that you use ([example](https://github.com/ashBabu/skidy/blob/ros2-ignition-gazebo/skidy_description/urdf/robot_test_diff_drive.urdf#L124)). By default, this topic name is something weird like `/robot_name/.../cmd_vel` and work only if you have properly set up the `ros-ign_bridge` to transfer msgs from Fortress to ROS

### Current issues
* If the differential drive plugin in activated (commenting out the odometry and velocity plugin) in [robot_urdf](skidy_description/urdf/robot_test_diff_drive.urdf),
publishing to `cmd_vel` steers the robot straight even though there is an angular velocity specified.

https://github.com/ashBabu/skidy/assets/10324110/137a58f3-55d4-4ce6-8c3f-e11993d74568

* The `odom` to `base_link` transform is missing with differential drive plugin
* Without a `joint_state_publisher` (already have `robot_state_publisher`), the `continuous` joints are not 
recognized by the tf-tree. Rviz2 shows a warning 

  ```
  [rviz2-6] Warning: Invalid frame ID "drivewhl_r_link" passed to canTransform argument source_frame - frame does not exist
  [rviz2-6] at line 93 in ./src/buffer_core.cpp
   ```
* Enabling the odometry and velocity plugin (commenting out diff drive) in [robot_urdf](skidy_description/urdf/robot_test_diff_drive.urdf),
behaves more or less like what a differential drive plugin should do

https://github.com/ashBabu/skidy/assets/10324110/b07d4f63-22a4-4ed3-aa1b-5db9b499bef7

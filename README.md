
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
Setup your colcon workspace

* `mkdir -p ~/skidy_ws/src`
* `cd skidy_ws/src`
* `git clone https://github.com/ashBabu/skidy.git`
* `git fetch origin --prune`
* `cd skidy`
* `git checkout ros2-ignition-gazebo`
* `cd ~/skidy_ws`
* `colcon build --symlink-install`
* `source install/setup.bash`

### Launch
* `ros2 launch skidy_bringup bringup_sim.launch.py` 
* Press the play button in Gazebo fortress

### To publish to cmd_vel topic
* `ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.5}"`
  *       OR
* `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"`

### Current issues
* If the differential drive plugin in activated (commenting out the odometry and velocity plugin) in [robot_urdf](skidy_description/urdf/robot_test_diff_drive.urdf),
publishing to `cmd_vel` steers the robot straingt even though there is an angular velocity specified.

  <video width="600" height="440" controls>
    <source src="videos/diff_drive_plugin.mp4" type="video/mp4">
  </video>

* The `odom` to `base_link` transform is missing with differential drive plugin
* Without a `joint_state_publisher` (already have `robot_state_publisher`), the `continuous` joints are not 
recognized by the tf-tree. Rviz2 shows a warning 
  ```
  [rviz2-6] Warning: Invalid frame ID "drivewhl_r_link" passed to canTransform argument source_frame - frame does not exist
  [rviz2-6] at line 93 in ./src/buffer_core.cpp
   ```
* Enabling the odometry and velocity plugin (commenting out diff drive) in [robot_urdf](skidy_description/urdf/robot_test_diff_drive.urdf),
behaves more or less like what a differential drive plugin should do

  <video width="600" height="440" controls>
    <source src="videos/odom_velocity_plugin.mp4" type="video/mp4">
  </video>
The tutorial is mostly from [theconstructsim](https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/)
[Git repository](https://github.com/mattborghi/m2wr_description)

### Step by step instructions to ROS Navigation stack
Setup your catkin workspace
* `mkdir -p skidy_ws/src`
* `cd skidy_ws/src`
* `git clone https://github.com/ashBabu/skidy.git`
* `cd ../../`
* `catkin_make`
* `source devel/setup.bash`

`001_skidy_full.xacro` is the very basic urdf with links and joints for a mobile robot. To view the robot,
* `sudo apt-get install ros-melodic-urdf-tutorial`
* `rosrun xacro xacro --inorder -o f001_skidy_full.urdf 001_skidy_full.xacro` # to get the urdf
* `roslaunch urdf_tutorial display.launch model:='$(find ros_nav)/urdf/001_skidy_full.urdf'`
OR
* `roslaunch skidy_description skidy_in_rviz.launch`  to view the robot in Rviz.


### Creating a gazebo world
Official tutorial available [here](https://classic.gazebosim.org/tutorials?tut=build_world&ver=1.9).

Fire up a terminal and type the following
* `gazebo`  
this will launch the an `empty_world` in `gazebo`. Add primitives (sphere, cuboid etc.) or use the `Insert` tab to create your own world. Save your world under `skidy_description/gazebo_worlds/simple_world.world`.
* `roslaunch skidy_description 002_gazebo_world.launch` # to launch this

### Spawn skidy in gazebo
This requires adding the following to the `002_gazebo_world.launch` and is given in `003_skidy_in_gazebo.launch`

` <node name="skidy_gazebo_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
      args="-urdf -param robot_description -model skidy -x $(arg x) -y $(arg y) -z $(arg z)" />
`
* `roslaunch skidy_description 003_skidy_in_gazebo.launch`  to view the robot in gazebo.

This will look as shown below

<img style="float: left;" title="skidy basic" src="skidy_navigation/images/skidy_in_gazebo.png" alt="" width="800" height="400"/>

### Add teleop_twist_keyboard
* ` sudo apt-get install ros-melodic-teleop-twist-keyboard`
* Add the following in the `urdf` or `skidy_gazebo.xacro`
```
<gazebo>
  <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
    <alwaysOn>true</alwaysOn>
    <updateRate>20</updateRate>
    <leftJoint>joint_left_wheel</leftJoint>
    <rightJoint>joint_right_wheel</rightJoint>
    <wheelSeparation>0.4</wheelSeparation>
    <wheelDiameter>0.2</wheelDiameter>
    <torque>0.1</torque>
    <commandTopic>cmd_vel</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>link_chassis</robotBaseFrame>
  </plugin>
</gazebo>
```
After launching the `003_skidy_in_gazebo.launch`, in another terminal run the following
* `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to control the robot via keyboard

**Note:** Might need to do `source devel/setup.bash`


### Navigation with skidy
Create a new catkin package `skidy_navigation`
* `cd skidy_ws/src/skidy/`
* `catkin_create_package skidy_navigation std_msgs sensor_msgs geometry_msgs roscpp rospy actionlib`
* `cd skidy_navigation && mkdir scripts`
Add the Python files `read_laser.py, obstacle_avoidance and go_to_point.py`. The explanations are given in the [tutorial]("/construct_tutorial/001_ Exploring ROS Using a 2 Wheeled Robot _ The Construct.pdf") from chapter 4-6.

### Gmapping with skidy
* Set the parameters in `skidy_navigation/launch/skidy_gmapping.launch`. The important ones are
`scan_topic, base_frame, odom_frame`. The details for the rest can be found [here](http://wiki.ros.org/gmapping).
* `roslaunch skidy_navigation skidy_gmapping.launch`. This will launch both `Rviz` and `gazebo` as the following lines are already added to the launch file
```
<include file="$(find skidy_description)/launch/001_skidy_in_rviz.launch">
    <arg name="model" value="$(arg model)"/>
</include>
<include file="$(find skidy_description)/launch/003_skidy_in_gazebo.launch">
    <arg name="model" value="$(arg model)"/>
</include>
```
* `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` # to control the robot using keyboard
* Add the following topics in `Rviz`
    * `LaserScan` with topic `skidy/laser/scan`
    * `RobotModel`
    * `Map` with topic `/map`
* Drive the robot using the keyboard to see the map building
* [Save the map](https://github.com/ashBabu/Utilities/wiki/ROS-Navigation#saving-the-map) using `rosrun map_server map_saver -f ~/skidy_ws/src/skidy/skidy_navigation/maps/map1`


This will look as shown below

<img style="float: left;" title="skidy basic" src="skidy_navigation/images/skidy_gmapping.png" alt="" width="800" height="400"/>

The tutorial is from [theconstructsim](https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/)

* `roslaunch skidy_description skidy_in_rviz.launch`  to view the robot in Rviz.
* `roslaunch skidy_description skidy_in_gazebo.launch`  to view the robot in gazebo.
* ` sudo apt-get install ros-melodic-teleop-twist-keyboard`

After launching the `skidy_in_gazebo.launch`, in another terminal run the following
* `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to control the robot via keyboard

**Note:** Might need to do `source devel/setup.bash`

## ROS Tutorials: Exploring ROS using a 2 wheeled Robot

From [m2wr_description](https://github.com/mattborghi/m2wr_description/blob/master/README.md)
### [Part 1: Explore the basics of robot modeling using the URDF](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/)

We are going to explore the basics of robot modeling using the Unified Robot Description Format (URDF) and we will have a model ready and running in Gazebo simulator.

* inside src/:
```sh$ catkin_create_pkg m2wr_description rospy```

* inside catkin_ws/:
```sh
$ catkin_make

$ source devel/setup.bash
```

* Launch rviz:
```sh
$ roslaunch m2wr_description rviz.launch
```

* Launch gazebo:
```sh
$ roslaunch gazebo_ros empty_world.launch
```

* Load our model:
```sh
$ roslaunch m2wr_description spawn.launch
```

* Run the following code in order to control the robot using the keyboard:
```sh
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### [Part 2: Explore the macros for URDF files using XACRO files](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part2)

We are going to explore the macros for URDF files, using XACRO files and we will have the same model organized in different files, in a organized way.

### [Part 3: Insert a laser scan sensor to the robot](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part3)

We are going to insert a laser scan sensor to a 2 wheeled robot the robot.

We will modify the urdf file (m2wr.xacro) as follows:

* Add a link element to our robot. This link will be cylindrical in shape and will represent the sensor.

* Add a joint element to our robot. This will connect the sensor to robot body rigidly.

* Define a new macro to calculate the inertial property of a cylinder using its dimensions (length and radius).

* Finally a laser scan sensor plugin element will add sensing ability to the link that we created (the cylinder representing the sensor).

* Having aded object in front of the robot in gazebo, open rviz and do the following:

	1. After starting rviz open the Graphical Tools window. Once rviz window loads you need to do the following settings

	2. Select odom in the Fixed Frame field (see the image below)

	3. Add two new displays using the Add button on the left bottom of rviz screen. The first display should be RobotModel and the other should be LaserScan

	4. Expand the LaserScan display by double clicking on its name and choose Topic as /m2wr/laser/scan

* Then move the robot and rviz should mark the sensor detecting this objects.

### [Part 4: Read the values of the laser scanner](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part4)

We are going to read the values of the laser scanner and filter a small part to work with.

* Clone the following repository, but just keep the 'my_worlds' folder:
```sh
$ cd simulation_ws/src

$ git clone https://marcoarruda@bitbucket.org/theconstructcore/two-wheeled-robot-simulation.git
```

The 'my_worlds' package consists of the following directories:

	1.	launch : Contains a launch file.

	2.	worlds : Contains multiple world description files.

We can run the world1 in Gazebo by writing:

```sh
$ roslaunch my_worlds world1.launch
```

Create a new catkin package named ```motion_plan``` with dependencies ```rospy```, ```std_msgs```, ```geometry_msgs``` and ```sensor_msgs```. Write the following commands:

```sh
$ cd ~/catkin_ws/src

$ catkin_create_pgk motion_plan rospy std_msgs geometry_msgs sensor_msgs

$ cd motion_plan

$ mkdir scripts

$ touch scripts/reading_laser.py
```

This reading_laser file will read the laser scan data coming on the /m2wr/laser/scan topic. Making the script executable:

```sh
$ cd ~/catkin_ws/src/motion_plan/scripts/

$ chmod +x reading_laser.py
```

To spawn our robot into the gazebo simulation write:

```sh
$ roslaunch m2wr_description spawn.launch

$ rosrun motion_plan reading_laser.py
```

[Part 5: An obstacle avoidance algorithm](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part5)

An obstacle avoidance algorithm will be explained.

Clone the following repository:

```sh
$ cd catkin_ws/src

$ git clone https://marcoarruda@bitbucket.org/theconstructcore/two-wheeled-robot-motion-planning.git
```
Launch the world02 scenario:

```sh
$ roslaunch my_worlds world2.launch
```
![](img/sensor.png)

To test the obstacle avoidance algorithm, spawn the robot into the world, as before and run the script:
```sh
$ rosrun motion_plan obstacle_avoidance.py
```

You can change various settings like speed of robot, sensing distance etc and see how it works.

[Part 6: Create an algorithm to go from a point to another](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part6)

We are going create an algorithm to go from a point to another using the odometry data to localize the robot.

We will use the concept of state machines to implement the navigation logic. In a state machine there are finite number of states that represent the current situation (or behavior) of the system. In our case, we have three states:

* Fix Heading: Denotes the state when robot heading differs from the desired heading by more than a threshold (represented by ```yaw_precision_``` in code)

* Go Straight: Denotes the state when robot has correct heading but is away from the desired point by a distance greater than some threshold ( represented by ```dist_precision_``` in code)

* Done: Denotes the state when robot has correct heading and has reached the destination.

The robot can be in any one state at a time and can switch to other states as different conditions arise. This is depicted by the following state transition diagram

![](img/diagram.png)

As made before, given that the world2 is already loaded, spawn the robot into the scenario and run the obstacle avoidance script:

```sh
$ rosrun motion_plan obstacle_avoidance.py
```

[Part 7: Work with wall following robot algorithm](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part7)

[Part 8: Work with the Bug 0 algorithm](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part8)

[Part 9: See the Bug 0 Foil](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part9)

[Part 10: Perform the motion planning task Bug 1](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part10)

[Part 11: From ROS Indigo to Kinetic](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part11)

[Part 12: Implement code for Bug 2 behavior](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part12)

[Part 13: Use ROS GMapping in our 2 wheeled robot](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part13)

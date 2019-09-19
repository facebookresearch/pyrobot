
# Examples for using PyRobot to control Sawyer


## Real robot hardware setup

Please follow the instructions of [ROS-UR page](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) to setup your robot hadware to work with ROS.

## Getting started

Make sure you have the necessary UR-ROS packages installed. Run the following command to do so,
On supported Linux distributions (Ubuntu, up to 16.04 (Xenial), `i386` and `amd64`) and ROS versions:

```bash
sudo apt-get install ros-$ROS_DISTRO-universal-robot
```

The official  UR5 tutorial on installation can be found [here](https://github.com/ros-industrial/universal_robot)




## Running the examples
1. Intial setup,

For real robot only,
```bash
roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]
```
**Note** Dont forget to the ```IP_OF_THE_ROBOT``` and ```REVERSE_PORT``` to match your robot's

For the Gazebo simulator only,
```bash
roslaunch ur_gazebo ur5.launch
```

2. Start the Moveit-Movegroup controller
```bash
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
```
Augument ```sim:=true``` for the above command for Gazebo only mode.

3. Run PyRobot examples in a new terminal

4. Optionally, you can also visualize the robot in R-Viz by the running this command,
```bash
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

# Examples for using PyRobot to control Sawyer


## Getting started

Make sure your system has [intera](http://sdk.rethinkrobotics.com/intera/Workstation_Setup#Install_ROS) installed. If not, please follow the [official tutorial](http://sdk.rethinkrobotics.com/intera/Workstation_Setup#Install_ROS) to setup intera and its [MoveIt](http://sdk.rethinkrobotics.com/intera/MoveIt_Tutorial) for Sawyer.

## Setup Sawyer-Gazebo simulator

To set up the gazebo simulator follow the "Sawyer Simulator Installation" instructions provided in this [tutorial](http://sdk.rethinkrobotics.com/intera/Gazebo_Tutorial).


## Running the examples
1. Intial setup,

For real robot only,
```bash
cd ~/ros_ws # or the appropriate catkin workspace in which intera_sdk package is in
 ./src/intera_sdk/intera.sh 
```

For the Gazebo simulator only,
```bash
cd ~/ros_ws # or the appropriate catkin workspace in which intera_sdk package is in
 ./src/intera_sdk/intera.sh sim
 roslaunch sawyer_gazebo sawyer_world.launch electric_gripper:=true #launch the Gazebo simulagtor
```

2. Start the joint trajectory controller
```bash
cd ~/ros_ws
./src/intera_sdk/intera.sh
rosrun intera_interface joint_trajectory_action_server.py
```

3. Launch MoveIt for Sawyer in a new terminal
```bash
cd ~/ros_ws
./src/intera_sdk/intera.sh
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
```

4. Run PyRobot examples in a new terminal

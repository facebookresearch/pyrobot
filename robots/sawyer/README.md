
# Examples for using PyRobot to control Sawyer


## Getting started

Make sure your system has [intera](http://sdk.rethinkrobotics.com/intera/Workstation_Setup#Install_ROS) installed. If not, please follow the [official tutorial](http://sdk.rethinkrobotics.com/intera/Workstation_Setup#Install_ROS) to setup intera and its [MoveIt](http://sdk.rethinkrobotics.com/intera/MoveIt_Tutorial) for Sawyer.

## Running the examples

1. Start the joint trajectory controller
```bash
rosrun intera_interface joint_trajectory_action_server.py
```

2. Launch MoveIt for Sawyer in a new terminal
```bash
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
```

3. Run PyRobot examples in a new terminal
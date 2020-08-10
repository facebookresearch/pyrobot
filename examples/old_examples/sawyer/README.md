
# Examples for using PyRobot API to control Sawyer


## Getting started

Make sure your system has [intera](http://sdk.rethinkrobotics.com/intera/Workstation_Setup#Install_ROS) installed. If not, please follow the [official tutorial](http://sdk.rethinkrobotics.com/intera/Workstation_Setup#Install_ROS) to setup intera and its [MoveIt](http://sdk.rethinkrobotics.com/intera/MoveIt_Tutorial) for Sawyer.

## Running the examples

1. Start the joint trajectory controller
```bash
cd ~/ros_ws
./intera.sh
rosrun intera_interface joint_trajectory_action_server.py
```

2. Launch MoveIt for Sawyer in a new terminal
```bash
cd ~/ros_ws
./intera.sh
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
```

3. Run PyRobot examples in a new terminal
```bash
cd ~/ros_ws
./intera.sh
source ~/pyenv_pyrobot/bin/activate
python joint_position_control.py
```

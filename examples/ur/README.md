
# Examples for using PyRobot API to control UR arm


## Getting started
Install ros package for UR arm
```
sudo apt-get install ros-kinetic-universal-robot
```
Sometime binary installation fails because of server issue. In this case you can try after some time or try to install from source. Instruction for installing from source can be found [here](https://github.com/ros-industrial/universal_robot).

If you install from source, make sure you activate the workspace where you have compiled the package before proceeding.

## Running the examples
In the following commands replace `X` with the robot `eg. [3, 3_e, 5, 5_e, 10, 10_e]
`
1. Connect to robot
```bash
roslaunch ur_bringup urX_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]
```

2. Launch MoveIt for robot in a new terminal
```bash
roslaunch urX_moveit_config ur5_moveit_planning_execution.launch limited:=true
```

3. Launch RViz with a configuration including the MoveIt! 
```bash
roslaunch urX_moveit_config ur5_moveit_planning_execution.launch limited:=true
```

4. Run PyRobot examples in a new terminal
```bash
source ~/pyenv_pyrobot/bin/activate
python move_to_neutral.py
```
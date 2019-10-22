
# Examples for using PyRobot API to control UR-e arm

## Note
Instructions are specifically for UR-e arm running 5.4 software version

## Getting started
Install ros package for UR-e arm
```
mkdir -p ~/ur_ws/src
cd ~/ur_ws/src
git clone -b kinetic_ur_5_4 https://github.com/AdmiralWall/ur_modern_driver.git
cd ..
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
catkin_make
echo "source ~/ur_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Setting up the robot
```
Need to add steps here
```

## Running the examples
In the following commands replace `X` with the robot `eg. [3_e, 5_e, 10_e]
`
1. Connect to robot
```bash
roslaunch ur_modern_driver urX_bringup_joint_limited.launch robot_ip:=IP_OF_THE_ROBOT use_lowbandwidth_trajectory_follower:=true
```

2. Launch MoveIt for robot in a new terminal
```bash
roslaunch urX_e_moveit_config ur5_e_moveit_planning_execution.launch
```

3. Launch RViz with a configuration including the MoveIt! 
```bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```

4. Run PyRobot examples in a new terminal
```bash
source ~/pyenv_pyrobot/bin/activate
python move_to_neutral.py
```
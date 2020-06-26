
# Examples for using PyRobot API to control UR-e arm

## Note
For real robot, instructions are specifically for `UR5-e` arm running `5.4` software version

## Getting started
Install ros package for UR-e arm
```
sudo apt-get install ros-kinetic-universal-robots
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

## Setting up the real robot
1. Power on the robot from UR control panel (its 3 step process)

On the main screen press `Power off` button on bottom left corner 
![image](https://drive.google.com/uc?export=view&id=1Wsd7lUug2NkAwbeKBT--u6oxg1_-KF4Y)

On the next screen press `ON` button
![image](https://drive.google.com/uc?export=view&id=1ZFs3g0JPkGlD1fP3Cyyd1lHTvrlDML8S)

On the next screen press `Start` button
![image](https://drive.google.com/uc?export=view&id=1Joo-v_vk2NiS_SCzzpBNNr8uZnrZwVQI)

After robot is powered on properly, screen will look like this
![image](https://drive.google.com/uc?export=view&id=110rn_BMLznj3QFXsrfLO3NTwPFZUapXZ)

2. Toggle Control to `Remote`. 

![image](https://drive.google.com/uc?export=view&id=1KNX6PZJetZsPCPGaLvwHhs8wEfVUlBBC)

## Running the examples

1. Connect to robot
* Real Robot
```bash
roslaunch ur_modern_driver ur5_e_bringup_joint_limited.launch robot_ip:=IP_OF_THE_ROBOT use_lowbandwidth_trajectory_follower:=true
```
* Simulator
```
roslaunch ur_gazebo ur5_bringup_joint_limited.launch
```

2. Launch MoveIt for robot in a new terminal
* Real Robot
```bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch
```
* Simulator
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
```

3. Launch RViz with a configuration including the MoveIt! 
```bash
roslaunch ur5e_moveit_config moveit_rviz.launch config:=true
```

4. Run PyRobot examples in a new terminal
```bash
source ~/pyenv_pyrobot/bin/activate
python move_to_neutral.py
```

## Things Missing
1. Velocity Control
2. Torque Control
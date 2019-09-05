---
id: manipulation_sawyer
title: Running basic tools for manipulation
sidebar_label: [Basic] Manipulation
---

In this example we will run through the basic manipulation tools currently available on PyRobot for Sawyer. 
Here is an example of what one can accoplish through this tutorial

<figure class="video_container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/DiaO8GyWyPs" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> 
</figure>

## Setup

To install the Sawyer software, please follow the instructions in this [README](https://github.com/facebookresearch/pyrobot/tree/master/robots/sawyer) to install and setup the appropriate sawyer software.

Go through the following steps to get the PyRobot code working on Sawyer.


<!--DOCUSAURUS_CODE_TABS-->
<!--Real Sawyer Robot-->
```bash
cd ~/ros_ws # or the appropriate catkin workspace in which intera_sdk package is in
./src/intera_sdk/intera.sh 
```

<!--Gazebo Sawyer Robot-->
```bash
cd ~/ros_ws # or the appropriate catkin workspace in which intera_sdk package is in
./src/intera_sdk/intera.sh sim
roslaunch sawyer_gazebo sawyer_world.launch electric_gripper:=true
```
<!--END_DOCUSAURUS_CODE_TABS--> 

In a new terminal:

<!--DOCUSAURUS_CODE_TABS-->
<!--Start the joint trajectory for Real Sawyer Robot-->
```bash
cd ~/ros_ws # or the appropriate catkin workspace in which intera_sdk package is in
./src/intera_sdk/intera.sh 
rosrun intera_interface joint_trajectory_action_server.py
```
<!--Start the joint trajectory for Gazebo Sawyer Robot-->
```bash
cd ~/ros_ws # or the appropriate catkin workspace in which intera_sdk package is in
./src/intera_sdk/intera.sh sim
rosrun intera_interface joint_trajectory_action_server.py
```
<!--END_DOCUSAURUS_CODE_TABS--> 

In a new terminal:

<!--DOCUSAURUS_CODE_TABS-->
<!--Launch MoveIt for Real Sawyer Robot-->
```bash
cd ~/ros_ws # or the appropriate catkin workspace in which intera_sdk package is in
./src/intera_sdk/intera.sh 
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
```
<!--Launch MoveIt for Gazebo Sawyer Robot-->
```bash
cd ~/ros_ws # or the appropriate catkin workspace in which intera_sdk package is in
./src/intera_sdk/intera.sh sim
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
```
<!--END_DOCUSAURUS_CODE_TABS--> 

Finally, make sure that the appropriate python virtual environment has been sourced before running any PyRobot package.

```bash
source ~/pyenv_pyrobot/bin/activate
```

## Basic movements

In your favorite Python command shell run the following to setup the robot object

<!--DOCUSAURUS_CODE_TABS-->
<!--Sawyer-->
```py
from pyrobot import Robot
import numpy as np
import time
import math

robot = Robot('sawyer',
	          use_base=False,
	          use_camera=False)
```
<!--END_DOCUSAURUS_CODE_TABS--> 

### Joint Position Control
<!--DOCUSAURUS_CODE_TABS-->
<!--Joint Position Control-->
```py
robot.arm.go_home()
target_joint = [0.704, -0.455, -0.159, 1.395, -1.240, 1.069, 2.477]
robot.arm.set_joint_positions(target_joint, plan=False)
robot.arm.go_home()
```
<!--END_DOCUSAURUS_CODE_TABS--> 
<figure class="video_container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/TJuXf3VExbE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

### Joint Velocity Control
<!--DOCUSAURUS_CODE_TABS-->
<!--Joint Velocity Control-->
```py
def sin_wave(t, f, A):
    return A * math.cos(2 * math.pi * f * t)

A = 0.2
f = 0.4
robot.arm.go_home()
start_time = time.time()
robot.arm.move_to_neutral()
while time.time() - start_time < 35:
    elapsed_time = time.time() - start_time
    vels = [sin_wave(elapsed_time, f, A)] * robot.arm.arm_dof
    robot.arm.set_joint_velocities(vels)
    time.sleep(0.01)
```
<!--END_DOCUSAURUS_CODE_TABS--> 
<figure class="video_container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/dwi1yAN3-vk" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

### Joint Torque Control
<!--DOCUSAURUS_CODE_TABS-->
<!--Joint Torque Control-->
```py
def spring_damping(position_err, velocity_err, spring_coef, damping_coef):
    torques = -np.multiply(spring_coef, position_err)
    torques -= np.multiply(damping_coef, velocity_err)
    return torques

ini_joint_angles = np.array(robot.arm.get_joint_angles())

spring_coef = np.array([30.0, 45.0, 15.0, 15.0, 9.0, 6.0, 4.5])
damping_coef = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
robot.arm.move_to_neutral()
while True:
    joint_angles = np.array(robot.arm.get_joint_angles())
    joint_velocities = np.array(robot.arm.get_joint_velocities())
    pos_err = joint_angles - ini_joint_angles
    vel_err = joint_velocities
    joint_torques = spring_damping(pos_err, vel_err, spring_coef, damping_coef)
    robot.arm.set_joint_torques(joint_torques)
    time.sleep(0.001)
```
<!--END_DOCUSAURUS_CODE_TABS--> 
This script implemented a virtual spring-damper control on the Sawyer manipulator. You can push the manipulator, and it will try to come back to the original pose.
<figure class="video_container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/TNbv-kO-0gw" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

### End-effector Pose Control

<!--DOCUSAURUS_CODE_TABS-->
<!--EE Pose Control-->
```py
target_poses = [{'position': np.array([0.8219, 0.0239, 0.0996]),
                 'orientation': np.array([[-0.3656171, 0.6683861, 0.6477531],
                                          [0.9298826, 0.2319989, 0.2854731],
                                          [0.0405283, 0.7067082, -0.7063434]])},
                {'position': np.array([0.7320, 0.1548, 0.0768]),
                 'orientation': np.array([0.1817, 0.9046, -0.1997, 0.3298])},
                ]
robot.arm.go_home()
time.sleep(1)
for pose in target_poses:
    robot.arm.set_ee_pose(plan=True, **pose)
    time.sleep(1)
```
<!--END_DOCUSAURUS_CODE_TABS--> 
<figure class="video_container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/xmaCYCSpGuU" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

### End-effector Cartesian Path Control

<!--DOCUSAURUS_CODE_TABS-->
<!--EE Cartesian Path Control-->
```py
plan = False
robot.arm.move_to_neutral()
time.sleep(1)
displacement = np.array([0.15, 0, 0])
robot.arm.move_ee_xyz(displacement, plan=plan)
time.sleep(1)
displacement = np.array([0., 0.15, 0])
robot.arm.move_ee_xyz(displacement, plan=plan)
time.sleep(1)
displacement = np.array([0., 0., 0.15])
robot.arm.move_ee_xyz(displacement, plan=plan)
time.sleep(1)
robot.arm.go_home()
```
<!--END_DOCUSAURUS_CODE_TABS--> 
<figure class="video_container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/ry3VUU4hXhE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>


### Moveit Planning with a New Obstacle (a Table) Added

You can manually add obstacle to the MoveIt planning scene so that MoveIt will make sure the planned path will not collide with the obstacle. Here we show an example of adding a table. You can see the table added visually in rviz.

<!--DOCUSAURUS_CODE_TABS-->
<!--Moveit with Obstacles-->
```py
from pyrobot.utils.util import MoveitObjectHandler
obstacle_handler = MoveitObjectHandler()
# Add a table
# position and orientation (quaternion: x, y, z, w) of the table
pose=[0.8,0.0,-0.23,0.,0.,0.,1.]
# size of the table (x, y, z)
size=(1.35,2.0,0.1)
obstacle_handler.add_table(pose, size)
target_poses = [{'position': np.array([0.8219, 0.0239, -0.1]),
                 'orientation': np.array([[-0.3656171, 0.6683861, 0.6477531],
                                          [0.9298826, 0.2319989, 0.2854731],
                                          [0.0405283, 0.7067082, -0.7063434]])},
                {'position': np.array([0.7320, 0.1548, -0.15]),
                 'orientation': np.array([0.1817, 0.9046, -0.1997, 0.3298])},
                ]
robot.arm.go_home()
time.sleep(1)
for pose in target_poses:
    robot.arm.set_ee_pose(plan=True, **pose)
    time.sleep(1)
robot.arm.go_home()
```
<!--END_DOCUSAURUS_CODE_TABS-->
<figure class="video_container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/DiaO8GyWyPs" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> 
</figure>
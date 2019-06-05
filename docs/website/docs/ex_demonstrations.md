---
id: demonstrations
title: Running demonstration tools for manipulation
sidebar_label: [Basic] Demonstration
---

You can also remotely teleoperate the robot to perform some complicated tasks or to collect demonstrations. This feature is only supported on LoCoBot right now. Here's an example of teleoperating the arm to stack some blocks.

<figure class="video_container">
	<iframe class="doc_vid" src="https://www.youtube.com/embed/Po42kt09FIM">
	</iframe>
</figure>

## Getting started

* Make sure robot is turned ON

* Start the python virtual environment

<!--DOCUSAURUS_CODE_TABS-->
<!--Sourcing virtual env-->
```bash
source ~/pyenv_pyrobot/bin/activate
```
<!--END_DOCUSAURUS_CODE_TABS--> 

* Launch the robot server. Note that you have to set `teleop:=true`. Pass in the corresponding flags when launching the robot.

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot Arm [Real]-->
```bash
# You will only be able to control the arm (no base and no camera)
roslaunch locobot_control main.launch use_arm:=true teleop:=true
```
<!--Arm+Base [Real]-->
```bash
# You will only be able to control the arm and the base
roslaunch locobot_control main.launch use_arm:=true use_base:=true teleop:=true
```
<!--Arm+Base+Camera [Real]-->
```bash
# You will only be able to control the arm, the base, and the camera
roslaunch locobot_control main.launch use_arm:=true use_base:=true use_camera:=true teleop:=true
```
<!--Arm+Base+Camera [Gazebo]-->
```bash
roslaunch locobot_control main.launch use_arm:=true use_sim:=true teleop:=true use_camera:=true use_base:=true
```
<!--END_DOCUSAURUS_CODE_TABS--> 

* Now we can run the teleoperation client (in a new terminal). The screen should display the relevant commands to control the arm. 
```bash
source ~/pyenv_pyrobot/bin/activate
cd ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_control/nodes
python keyboard_teleop_client.py
```
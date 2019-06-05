---
id: pushing
title: Running pushing example on your robot
sidebar_label: [Basic] Pushing
---

Here is a demo video showing what one can accomplish through this tutorial.
<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/p9NNsDWe9sg" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

## Get Started

* Make sure robot is turned ON

* Launch the robot driver
<!--DOCUSAURUS_CODE_TABS-->
<!--Launch driver-->
```bash
roslaunch locobot_control main.launch use_arm:=true use_camera:=true
```
<!--END_DOCUSAURUS_CODE_TABS--> 

* Run the pushing example

`pushing.py` script accepts two parameters: `floor_height` and `gripper_len`. You need to tune these two parameters a bit. `floor_height` means the `z`-coordinate of the floor. Points with `z`-coordinate smaller than
this value will be filtered. Only points heigher than the floor will be used for clustering. `gripper_len`
means the length of the gripper (from `gripper_link` to the tip of the gripper). You may need to tune these
two parameters when you run the example. 

If the gripper goes down too much and it hits the floor while pushing the object, you can make the `gripper_len` bigger. If the gripper doesn't go down enough and it doesn't touch the object, you can try making the `gripper_len` smaller.

To tune `floor_height`, you need to run the following script first.
<!--DOCUSAURUS_CODE_TABS-->
<!--Visualize filtered point cloud-->
```bash
source ~/pyenv_pyrobot/bin/activate
cd ~/low_cost_ws/src/pyrobot/examples/locobot/manipulation
python realtime_point_cloud.py --floor_height=0.01
```
<!--END_DOCUSAURUS_CODE_TABS--> 

This script shows the point cloud in the scene after filtering (points with depth more than 1.5m and points with `z` coordinate (height) larger than `floor_height` will be removed). The remaining point cloud will be used for clustering and pushing in `pushing.py`. So you need to tune the `floor_height` such that the floor is completely removed (which means its value cannot be too small) and as much points of the the objects as possible are remained (which means its value cannot be too big).

After tuning, you can run the pushing script in a new terminal as follows:

<!--DOCUSAURUS_CODE_TABS-->
<!--Run pushing script-->
```bash
source ~/pyenv_pyrobot/bin/activate
cd ~/low_cost_ws/src/pyrobot/examples/locobot/manipulation
python pushing.py --floor_height=0.01 --gripper_len=0.12
```
<!--END_DOCUSAURUS_CODE_TABS--> 


**Warning**: we are using the analytical inverse kinematics in this example, which means
no collision checking is performed here. So the robot arm may hit the robot itself sometimes.

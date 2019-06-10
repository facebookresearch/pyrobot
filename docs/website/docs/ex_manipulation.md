---
id: manipulation
title: Running basic tools for manipulation
sidebar_label: [Basic] Manipulation
---

In this example we will run through the basic manipulation tools currently available on PyRobot. 

Here is a demo video showing what one can accomplish through this tutorial.
<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/lzuVGpJnnDY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

## Setup
This tutorial can also be run using a simulated version of the robot. Before we get started with this, ensure the following:

* The robot arm is supported by PyRobot. Check if your robot is supported [here](gs_overview.md).

* The robot arm is switched ON. With the LoCoBot this is done by connecting the power supply and USB to the arm.

* The appropriate python virtual environment has been sourced before running any PyRobot package.

```bash
source ~/pyenv_pyrobot/bin/activate
```


<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot Setup Instructions-->

LoCoBot's launch file has been run. Note that you have to set `use_arm:=true`.

```bash
roslaunch locobot_control main.launch use_arm:=true
```

Similar to the real robot, for LoCoBot gazebo simulator, run the following command,
```bash
roslaunch locobot_control main.launch use_arm:=true use_sim:=true
```
<!--END_DOCUSAURUS_CODE_TABS--> 




## Basic movements

In your favorite Python command shell run the following to setup the robot object

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot-->
```py
from pyrobot import Robot
import numpy as np
robot = Robot('locobot')
```
<!--LoCoBot-Lite-->
```py
from pyrobot import Robot
import numpy as np
robot = Robot('locobot_lite')
```
<!--END_DOCUSAURUS_CODE_TABS--> 

This creates a `Robot` object that encapsulates the robot's basic motion utilities. 

### Joint control

To move the joints of the robot to a desired joint configuration, run the following snippet:

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot-->
```py
target_joints = [
        [0.408, 0.721, -0.471, -1.4, 0.920],
        [-0.675, 0, 0.23, 1, -0.70]
    ]
robot.arm.go_home()

for joint in target_joints:
    robot.arm.set_joint_positions(joint, plan=False)
    time.sleep(1)
    
robot.arm.go_home()
```
`Robot.arm.go_home()` makes the arm to move to its *home* position. Since we are using a 5-joint (DoF, degree-of-freedom) arm on the LoCoBot, the `target_joint` is a 5D vector of desired individual joint angles from the base of the arm to its wrist. Then finally through the `
set_joint_positions` method the Robot will move to the desired `target_joint`. The `plan=False` argument means that the robot will not use MoveIt to plan around obstacles (like the base or the arm itself). To plan around obstacles, look at using [MoveIt](#planning-using-moveit).
<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/0GKUqgmJuDM" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>  


<!--END_DOCUSAURUS_CODE_TABS--> 





### End-effector pose control

In this example, we will look at controlling the [end-effector](https://whatis.techtarget.com/definition/end-effector) pose of the arm. A *pose* object has two components: *position* and *rotation*. *Position* is a 3D numpy array representing the desired position. *Orientation* can be a rotation matrix [3x3], euler angles [3,], or quaternion [4,]. 

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot-->

```py
import time
target_poses = [{'position': np.array([0.279, 0.176, 0.217]),
                 'orientation': np.array([[0.5380200, -0.6650449, 0.5179283],
                                          [0.4758410, 0.7467951, 0.4646209],
                                          [-0.6957800, -0.0035238, 0.7182463]])},
                {'position': np.array([0.339, 0.0116, 0.255]),
                 'orientation': np.array([0.245, 0.613, -0.202, 0.723])},
                ]
robot.arm.go_home()

for pose in target_poses:
    robot.arm.set_ee_pose(**pose)
    time.sleep(1)
robot.arm.go_home()
```

Note that since the LoCoBot only has 5 DoFs, it can only reach target poses that lie in its configuration space. Check the API for more on how to use the method `set_ee_pose`.

<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/lzuVGpJnnDY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

<!--END_DOCUSAURUS_CODE_TABS--> 


### End-effector Position and Pitch Roll Control

**This is a LoCoBot-specific example.** 

As LoCoBot is a 5-DOF robot, you can specify its pose with an end-effector position (x,y,z), a pitch angle and a roll angle (no need to specify yaw angle as it only has 5 degrees of freedom).

```py
target_poses = [
    {'position': np.array([0.28, 0.17, 0.22]),
     'pitch': 0.5,
     'numerical': False},
    {'position': np.array([0.28, -0.17, 0.22]),
     'pitch': 0.5,
     'roll': 0.5,
     'numerical': False}
]

robot.arm.go_home()

for pose in target_poses:
    robot.arm.set_ee_pose_pitch_roll(**pose)
    time.sleep(1)

robot.arm.go_home()
```
<figure class="video_container">
<iframe class="doc_vid" src="https://www.youtube.com/embed/YOYku4IqZBc" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

### End-effector Cartesian Path control

In this example, we will move the arm in the X,Y,Z coordinates in straight-line paths from the current pose.

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot-->

```py
robot.arm.go_home()
displacement = np.array([0, 0, -0.15])
robot.arm.move_ee_xyz(displacement, plan=True)
robot.arm.go_home()
```
If `plan=True`, it will call the internal cartesian path planning in [MoveIt](#planning-using-moveit). 

If `plan=False`, it will simply perform linear interpolation along the target straight line and do inverse kinematics (you can choose whether you want to use the numerical inverse kinematics or analytical inverse kinematics by passing `numerical=True` or `numerical=False`) on each waypoints. Since LoCoBot is a 5-DOF robot, the numerical inverse kinematics sometimes fail to find the solution even though there exists a solution. So analytical inverse kinematics might work better in such cases.

<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/s030tLu2oZs" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>


<!--END_DOCUSAURUS_CODE_TABS--> 


### Joint torque control

**Warning for usage.** Each though each joint accepts the torque command, there is no gravity compensation implemented for LoCoBot yet. Torque control mode is not recommended for LoCoBot and it's not well tested. Sawyer does support torque control and it's well tested.

For direct torque control, one can use the `set_joint_torques` method as follows. Firstly, you will need to kill the previously launched driver and relaunched it with the following command. 

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot Only-->
```bash
roslaunch locobot_control main.launch use_arm:=true torque_control:=true
```
<!--END_DOCUSAURUS_CODE_TABS--> 

Then you can use the following command to send torque values to robots. Try to keep the arm in initial condition as shown in the below video, as the behavior will be different for a different configuration. For this example, we are going to apply torque only on joint 4. This will move robot joint 4 to the extreme. After completion, 
the joint will be free again. The torque requirements may vary from robot to robot. So if joint 4 doesn't move using following script, try to apply a higher magnitude of torque.
<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot-->
```py
from pyrobot import Robot
import time
arm_config = dict(control_mode='torque')
robot = Robot('locobot', arm_config=arm_config)
target_torque = 4 * [0]

target_torque[3] = -0.45
robot.arm.set_joint_torques(target_torque)
time.sleep(2)

target_torque[3] = 0.0
robot.arm.set_joint_torques(target_torque)
```
<!--END_DOCUSAURUS_CODE_TABS--> 
<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/dIrN-wUGqao" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>


### Gripper control

Opening and closing the gripper is done via the `gripper` object.

```py
import time

robot.gripper.open()
time.sleep(1)

robot.gripper.close()
```

<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/d1VmHkOAIT0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

## Planning using MoveIt

To avoid hitting obstacles like the base or the arm itself, we support planning via [MoveIt!](https://moveit.ros.org/). To use this, we need to first set the robot with the approriate planning parameters:

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot-->
```py
config = dict(moveit_planner='ESTkConfigDefault')
robot = Robot('locobot', arm_config=config)
```
<!--END_DOCUSAURUS_CODE_TABS--> 

After this run `set_joint_positions` with the argument `plan=True`.

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot-->
```py
target_joints = [
        [0.408, 0.721, -0.471, -1.4, 0.920],
        [-0.675, 0, 0.23, 1, -0.70]
    ]
robot.arm.go_home()

for joint in target_joints:
    robot.arm.set_joint_positions(joint, plan=True)
    time.sleep(1)

robot.arm.go_home()
```
<!--END_DOCUSAURUS_CODE_TABS--> 

<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/fr-9SrY00YI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

## Sensing

Most arms come with proprioceptive feedback. The following functions can be used to read the current state of the robot (joint angles, velocities, torques), etc.

```py
# Get all joint angles of the robot
current_joints = robot.arm.get_joint_angles()

# Get state of a specific joint
current_joint = robot.arm.get_joint_angle("joint_5")

# Get all joint velocities of the robot
current_velocity = robot.arm.get_joint_velocities()

# Get current joint torques (if mode='torque')
current_torques = robot.arm.get_joint_torques()
```

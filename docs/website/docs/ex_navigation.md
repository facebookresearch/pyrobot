---
id: navigation
title: Running navigation tools on your robot
sidebar_label: [Basic] Navigation
---

In this example we will run through the basic navigation tools currently available on PyRobot. 

Here is a demo video showing what one can accomplish through this tutorial.
<figure class="video_container">
	<iframe class="doc_vid" src="https://www.youtube.com/embed/vKpaujZYcOM" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>


## Setup
This tutorial can also be run using a simulated version of the robot. Before we
get started with this, ensure the following:

* The robot base is supported by PyRobot. Check if your robot is supported [here](gs_overview.md).

* The robot base is switched ON. With the LoCoBot the base beeps when connected.

* The appropriate python virtual environment has been sourced.
```bash
source ~/pyenv_pyrobot/bin/activate
```

* The robot's launch file has been run. Note that you have to set `use_base:=true`.

<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot[Real Robot]-->
```bash
roslaunch locobot_control main.launch use_base:=true
# for LoCoBot-Lite append the command above with 'base:=create'
```
<!--LoCoBot[Simulator]-->
```bash
roslaunch locobot_control main.launch use_base:=true use_sim:=true 
# for LoCoBot-Lite append the command above with 'base:=create'
```
<!--END_DOCUSAURUS_CODE_TABS-->

## Base State
Base state is represented by `[x, y, yaw]`, the `x`-coordinate, `y`-coordinate
of the `base_link` and the robot heading `yaw`. By default this state is
estimated via intertial sensors, or wheel encoders. This state can be queried
as follows (You can run the following code in a **python** terminal):
```py
#import appopriate dependencies
from pyrobot import Robot

# Create robot.
robot = Robot('locobot')

# Get the current pose of the base i.e, a state of the form (x, y, yaw). Note
# that over here we are querying the 'odom' state, which is state estimated
# from inertial sensors and wheel encoders on the base.
current_state = robot.base.get_state('odom')

# (Advanced) If you are running visual SLAM, then you can also query the state
# as estimated from visual sensors, using the following. State estimates from
# visual SLAM can be more accurate.
current_state = robot.base.get_state('vslam')
```

## Basic Control
In this section, we will talk about two modes of controlling the LoCoBot base-
Velocity control and Position control. One should note the same tutorials also
apply to LoCoBot-Lite with very minimal change to the code. 

We will talk about velocity and position control by introducing examples.

### Velocity control
This mode of control allows us to command the base with a particular linear and
angular velocity for a specified ammount of time.

An example script that performs velocity control,
```py

#import appopriate dependencies
from pyrobot import Robot

# Create the Robot object that interfaces with the robot.
robot = Robot('locobot') 
# If you want to use LoCoBot-Lite instead, replace the argument 'locobot' with
# 'locobot_lite'

# linear_velocity in m/s
linear_velocity = 0.1 

# rotational_velocity in radian / s
rotational_velocity = 0.5 

# execution_time in seconds
execution_time = 4 

# Command to execute motion
robot.base.set_vel(fwd_speed=linear_velocity, 
                   turn_speed=rotational_velocity, 
                   exe_time=execution_time)

# To stop the robot at any time:
robot.base.stop() 
```

### Frames of reference
As shown in the figure below, there are two frames of reference available for the base - Local and Global frames.

<iframe src="https://drive.google.com/file/d/1O4XnCoYIdZmJBgtkbILo67PI9EcLkE6I/preview" width="640" height="480"></iframe>

Local frame is the frame of reference attached to the base of the robot and moves with it as the robot moves i.e, all the points in this frame are relative to the robot. 

Global frame is a stationary frame of reference. It is the initial frame that the robot started at.

### Position control
This mode of control allows us to command the base to go to a specified target
(of the form `[x, y, yaw]`) in the environment.

We currently support three different base-controllers for position control:
`ILQR`, `Proportional` and `Movebase`. 

Following code shows an example of position control.
```py
from pyrobot import Robot

# base_config_dict is a dictionary that contains different base configuration
# parameters. 'base_controller' can be set to 'ilqr' or 'proportional' or
# 'movebase' to use the respective controllers.
base_config_dict={'base_controller': 'ilqr'} 

# crate the Robot object with the specified base_config
robot = Robot('locobot', base_config=base_config_dict)

# target position we want the base to go to
target_position = [1,1,0.5] # this is a 2D pose of the form [x, y, yaw]

# Now command the robot to go to the target pose in the enviroment
# 'go_to_absolute' assumes that the target is in world frame.
robot.base.go_to_absolute(target_position) 

# Targets can be specified in robot's coordinate frame.
# robot.base.go_to_relative(target_position)
```

As shown below, we can also modify `go_to_absolute` or `go_to_relative`
function arguments to enable or disable different features.
```py
# smooth ensures that the robot only follows smooth motions while going to goal
# smooth mean no on-spot rotations.
robot.base.go_to_absolute(target_position, smooth=True) 

# close_loop ensures that the controller acts in closed loop by using onboard
# odommetry
robot.base.go_to_absolute(target_position, close_loop=True) 
```

Below are few different position control examples.
<!--DOCUSAURUS_CODE_TABS-->
<!--Example 1 -->
```py
from pyrobot import Robot
base_config_dict={'base_controller': 'proportional'}
robot = Robot('locobot', base_config=base_config_dict)
target_position = [1.0, 0.0, 0.0] # go forward 1 meter
robot.base.go_to_relative(target_position, smooth=False, close_loop=True)
```
<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/iN0hW9XmLCc" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>
<!--Example 2 -->
```py
from pyrobot import Robot
base_config_dict={'base_controller': 'ilqr'}
robot = Robot('locobot', base_config=base_config_dict)
target_position = [-1.0, 0.0, 0.0] # go reverse 1 meter
robot.base.go_to_relative(target_position, smooth=False, close_loop=True)
```
<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/f7072CazsNM" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>
<!--Example 3 -->
```py
from pyrobot import Robot
base_config_dict={'base_controller': 'movebase'}
robot = Robot('locobot', base_config=base_config_dict)
target_position = [0.0, 0.0, 1.5707] # rotate on-spot by 90 degrees
robot.base.go_to_relative(target_position, smooth=False, close_loop=True)
```
<figure class="video_container">
 <iframe class="doc_vid" src="https://www.youtube.com/embed/U-GkUq2jnCI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>
<!--Example 4 -->
```py
from pyrobot import Robot
base_config_dict={'base_controller': 'ilqr'}
robot = Robot('locobot', base_config=base_config_dict)
target_position = [1.0, 1.0, 0.0] 
robot.base.go_to_relative(target_position, smooth=False, close_loop=True)
robot.base.go_to_relative(target_position)
```
<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/F6TOQG0NSgo" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>
<!--END_DOCUSAURUS_CODE_TABS-->

### Trajectory Tracking
Feedback controllers implemented in the library can be used to close the loop
on a given state-space trajectory. The base state is characterized by `[x, y,
yaw]`, the `x`-coordinate, `y`-coordinate and the robot heading `yaw`. Given a
state-space trajectory (at the frequency at which the tracker is run), the
`track_trajectory` function can close the loop on the trajectory.

```py
from pyrobot import Robot
import numpy as np
base_config_dict={'base_controller': 'ilqr'}
robot = Robot('locobot', base_config=base_config_dict)

# Generate a straight line trajectory, for the robot to track. Note that this
# trajectory needs to be generated at the frequency of the feedback controller
# that will track it.

# We need to generate a trajectory at the following rate.
dt = 1. / robot.configs.BASE.BASE_CONTROL_RATE

# We will generate a trajectory 1m long and track it such that the robot moves
# at half the max speed of the robot.
v = robot.configs.BASE.MAX_ABS_FWD_SPEED / 2.

# distance moved per time step
r = v * dt

# number of time steps 
t = int(1/r)

# initial state
x, y, yaw = robot.base.get_state('odom') 

# generate state trajectory
states = []
for i in range(t): 
	states.append([x+r*np.cos(yaw)*i, y+r*np.sin(yaw)*i, yaw])
states = np.array(states)

# Call trajectory tracker
robot.base.track_trajectory(states, close_loop=True)
```
In this example, we tracked a very simple trajectory, but the implementation
can track more complex trajectories as well. More a more advanced example see
[here](https://github.com/facebookresearch/pyrobot/blob/master/examples/locobot/navigation/base_trajectory_tracking.py).

## Position control with map (Real robot)
Position control with map is an enhanced position control feature that allows
us to leverage the map contructed by the onboard SLAM algorithms while going to
a specific target in the environement. This feature allows to avoid obstacles
in the environment while going to a target postion as the robot only travels
through the free space deemed worthy by the onboard SLAM.

To use this feature we need to modify the initially specified launch file arguments as follows,
<!--DOCUSAURUS_CODE_TABS-->
<!--LoCoBot[Real Robot]-->
```bash
roslaunch locobot_control main.launch use_base:=true use_vslam:=true use_camera:=true
```
<!--LoCoBot-LITE[Real Robot]-->
```bash
roslaunch locobot_control main.launch use_base:=true base:=create use_vslam:=true use_camera:=true
```
<!--END_DOCUSAURUS_CODE_TABS-->
Note that this feature only works on the **real robot**.

---

**Warning:**

1. After running the above launch command,
please do not move in front of the camera or temporarily block the camera view. These actions would be registered as permanent obstacles by SLAM which is running perpetualy in the background. The SLAM algorithm here does not deal with the dynamic obstacles.

2. SLAM only works in environments that have rich RGB feature points and could fail otherwise.

3. While using PyRobot, if you launch the robot with different settings, you need to exit the python terminal,
and import PyRobot in a new python terminal.
---

Here is an example showing position control with map in action.
```py
from pyrobot import Robot

# 'base_planner' is the planner used to generate collision free plans to goal
base_config_dict={'base_controller': 'proportional', 'base_planner':'movebase'}

robot = Robot('locobot', base_config=base_config_dict)

# 'use_map' argument determines if map should be used or not in position control.
robot.base.go_to_relative([2.0, 0.0, 0.0], use_map=True, smooth=False, close_loop=True)
```
<figure class="video_container">
  <iframe width="560" height="315" src="https://www.youtube.com/embed/YKzqxi_ATsA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>




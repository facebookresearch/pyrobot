---
id: next-manipulation
title: PyRobot Next Version as a Modular Framework
sidebar_label: PyRobot Next Version Overview
---

In this example we will run through a basic manipulation example to demonstrate PyRobot as a modular framework.

This tutorial is using a simulated version of the robot. Before we get started with this, ensure the following:

* The appropriate python virtual environment has been sourced.
```bash
load_pyrobot_env
```

# Creating World

A `World` represent an environment with four types of modules: robots, sensors, algorithms, and obstacles.

```py
#import appopriate dependencies
from pyrobot import World

# Create world.
world =  World(config_name='env/locobot_arm_env.yaml')
```

Now we have created a default manipulation world object configured by `{$path-to-pyrobot-root}/pyrobot/src/pyrobot/hydra_config/env/locobot_arm_env.yaml`. Let's take a look at what's inside this config file.

```yaml
environment:
  world_frame: 'map'

  # A locobot is instantiated at [0,0,0] in the world
  robots:
    - robot: 'locobot'
      label: 'locobot'
      ...

  # There's no sensor and obstacles in the environment
  sensors: null

  obstacles: null

  # 4 Algorithms are instantiated with the world
  algorithms:
    ...
    - algorithm: 'locobot_moveit_kin_planner'
      label: 'moveit_planner'
      ...

    - algorithm: 'locobot_kdl_kinematics'
      label: 'kdl_kinematics'
      ...

  # manages roslaunch files: this chuck specify the world would be spinning up in gazebo mode (simulation).
  ros_launch:
    ns: ''
    name: 'world'
    mode: gazebo
    habitat: null
    gazebo: null
    realrobot: null
```

Here we initiated a world object with a robot module with label `locobot` configured by `{$path-to-pyrobot-root}/pyrobot/src/pyrobot/hydra_config/robot/locobot.yaml`, and two algorithm module configured by `{$path-to-pyrobot-root}/pyrobot/src/pyrobot/hydra_config/algorithm/{$algorithm-name}.yaml` under mode `gazebo`.

# Robot Control

We can access locobot by:

```py
bot = world.robots['locobot']
```

Let's also take a look at what's inside `locobot.yaml`.

```yaml
name: 'locobot'

ros_launch:
  mode: gazebo
  name: ${name}
  wait: 30
  ...
  gazebo:
    # roslaunch command for spinning up locobot in gazebo mode
    common: "roslaunch locobot_control main.launch use_sim:=true"
    ...
...
modules:
  - object:
      _target_: pyrobot.robots.locobot.arm.LoCoBotArm
    ...
    name: 'arm'
    conf:
      ...

  - object:
      _target_: pyrobot.robots.locobot.base.LoCoBotBase
    ...
    name: 'base'
    conf:
      ...

  - object:
      _target_: pyrobot.robots.locobot.gripper.LoCoBotGripper
    ...
    name : 'gripper'
    conf:
      ...

  - object:
      _target_: pyrobot.robots.locobot.camera.LoCoBotCamera
    ...
    name : 'camera'
    conf:
      ...
```

`locobot` is a dictionary consists of four components: `arm`, `base`, `gripper` and `camera`, which extend `Arm`, `Base`, `Gripper` and `Camera` base class respectively. Therefore, they are guaranteed to have the same interface defined by the base class, for example, `arm` would have `go_home`, `get_joint_positions/velocities/torque`, `set_joint_positions/velocities/torque` as its member function, and their behavior would be the same across hardware. We can query locobot arm component by:

```py
joint = [0.408, 0.721, -0.471, -1.4, 0.920]
arm = bot['arm']
arm.set_joint_positions(joint)
arm.go_home()
```

# Algorithm Control

Similarly, we can access the planner by:

```py
planner = world.algorithms['moveit_planner']
```

Let's now take a look at its configuration.

```yaml
name: 'locobot_moveit_kin_planner'

ros_launch:
  mode: test
  ...

  test:
    common: "roslaunch locobot_moveit_config move_group.launch allow_trajectory_execution:=true fake_execution:=false info:=true" # write corresponding roslaunch command here

algorithm:
  _target_: pyrobot.algorithms.motion_planner_impl.moveit_kin_planner.MoveitKinPlanner # object definition path

robot_labels: ['locobot'] # name of the robot it would be interact with

conf: # any algorithm specific configs
  IK_POSITION_TOLERANCE: 0.005
  IK_ORIENTATION_TOLERANCE: 0.05

dependencies:
  - algorithm: 'locobot_kdl_kinematics'
```

Same as `LoCoBotArm` object, `MoveitKinPlanner` also extends `MotionPlanner` base class, and therefore guarenteed to have member functions `plan_end_effector_pose`, `plan_joint_angles` and `compute_cartesian_path`.

```py
pos = np.array([0.339, 0.0116, 0.255])
ori = np.array([0.245, 0.613, -0.202, 0.723]),
planner.plan_end_effector_pose(pos, ori)
```

# Plug-in-and-play Algorithms

If we look into the implementation of `MoveitKinPlanner`, we'll see the member function `plan_end_effector_pose` relies on a kinematic module to compute ik from given pose. In config file the default kinematic module is set to `locobot_kdl_kinematics`, but you can query it using its base class name `"Kinematics"`.

```py
from pyrobot.algorithms.kinematics import Kinematics
isinstance(planner['Kinematics'], Kinematics)
>>> True
```

Using base class name suggests you can substitute such dependency with any other algorithm extending the same class and get similar behavior.

```py
from pyrobot import make_algorithm
old_kinematics = planner['Kinematics']
new_kinematics = make_algorithm("algorithm/locobot_kdl_kinematics") # making a new copy

planner['Kinematics'] = new_kinematics
planner.plan_end_effector_pose(pos, ori) # same behavior!

print(planner['Kinematics'] is old_kinematics)
>>> False
```

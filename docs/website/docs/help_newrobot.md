---
id: new_robot_support
title: Interfacing a new robot with PyRobot
sidebar_label: New Robot Support
---

In the following context, we will use `<ROBOT_NAME>` to represent the robot name.
## Create a new configuration file

The first step for interfacing with a new robot is to create a new configuration file. The configuration file name should follow the following naming convention: `<ROBOT_NAME>_config.py`. The easiest way to create this file is to inherit the pre-defined configurations in `src/pyrobot/cfg/config.py`. In `config.py`, we have defined some configurations that are required in the `Robot` and `ARM` class. Each configuration here should be given the proper values.

An example of how to write the configuration file can be found in `pyrobot/cfg/sawyer_config.py`. `sawyer_config.py` inherits the configurations defined in `config.py` and changes the default values for these configurations. `sawyer_config.py` also shows how to add more configurations specific to the new robot in the configuration file.

## Inherit the PyRobot parent classes

The next step is to inherit the existing PyRobot parent classes (Arm, Gripper, Camera, Base). 

* Create a folder in `src/pyrobot` named as `<ROBOT_NAME>`. 
* Create a python script named as `arm.py`, `gripper.py`, `base.py`, or `camera.py` depending on what hardwares the new robot have. For example, if the new robot has all these 4 components, then you should create 4 files above. If the new robot only has arm and gripper, you should only create `arm.py` and `gripper.py`. 
* In each file, create a new class (the class name should be consistent with the configuration `CLASS` in `<ROBOT_NAME>_config.py`) and inherit the PyRobot parent class.

* You should reuse as many methods as possible in the parent class. If the new robot does not support a method, you can overwrite the method and raise an error if the method is being called. Overall, the library is very flexible and you can overwrite any method. An example of writing an `arm.py` can be found in `src/pyrobot/sawyer`. In the case of Sawyer robot, we only need to overwrite the command publish functions to make the code compatible with pyrobot.

## Write unit tests

After finishing the code, you should write unit tests for the new robot, if possible. Examples of unit tests can be found in `tests/`.

## Add examples

You are suggested to add examples in `examples/`. Create a new folder called `<ROBOT_NAME>`, and add your example files.


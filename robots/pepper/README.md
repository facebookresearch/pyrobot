---
id: setup_pepper
title: PyRobot setup for Pepper
sidebar_label: [Basic] Setup
--

## Prerequisite
Make sure that [ROS](http://wiki.ros.org/ROS/Installation) is installed on your computer.

## ROS packages for Pepper
You will first need to install the ROS packages for the Pepper robot:
```bash
# Replace distribution by melodic, kinetic or indigo, and robot by pepper or nao
sudo apt-get install ros-distribution-robot-meshes

# For Pepper with ROS kinetic:
sudo apt-get install ros-kinetic-pepper-meshes
```

You will then need to install __naoqi_libqi__ and __naoqi_libqicore__:
```bash
# Replace distribution by melodic, kinetic or indigo
sudo apt-get install ros-distribution-naoqi-libqi
sudo apt-get install ros-distribution-naoqi-libqicore
```

You can then install naoqi_driver (bridge between ROS and NAOqi) and naoqi_bridge_msgs. You can install those packages using apt-get (as specified below), but you can of course build those packages from source:
```bash
# Replace distribution by melodic, kinetic or indigo
sudo apt-get install ros-distribution-naoqi-driver
sudo apt-get install ros-distribution-naoqi-bridge-msgs
```

## Working with a real robot
Before launching any PyRobot script, run the naoqi_driver package:
```bash
# If you intalled the driver from source, make sure that you sourced the catkin workspace
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=robot_ip network_interface:=interface
```

This will launch the naoqi_driver ROS package and connect it to an existing Pepper. You have to specify two parameters when launching the driver:
* __nao_ip__: The ip address of the robot
* __network_interface__: The network interface used (eth0, wlan1, ...). Use the ifconfig command if you're unsure about the name of the interface

You can then launch your PyRobot scripts, or the one provided in the _examples/pepper_ folder.


## Working with a simulated robot
The [qiBullet simulator](https://github.com/softbankrobotics-research/qibullet) can be used to instanciate a virtual Pepper robot.

The qiBullet simulator can be installed via pip (Python 2 or 3):
```bash
pip install qibullet
```

Please note that the installation of the additional simulator resources will automatically be triggered if you try to spawn a Pepper, NAO or Romeo for the first time (to enable the installation, you will need to accept a license, more information about the installation process can be found on the [qiBullet wiki](https://github.com/softbankrobotics-research/qibullet/wiki/Tutorials:-Installation)).

To create a virtual Pepper and connect it to ROS, you will first need to create roscore from a terminal:
```bash
roscore
```

From another terminal, execute the __qibullet_launcher.py__ script:
```bash
python qibullet_launcher.py
```

By default, only the top camera of the robot is subscribed to, you can modify the `qibullet_launcher.py` script to subscribe to other cameras (bottom or depth cameras, more informations [here](https://github.com/softbankrobotics-research/qibullet/wiki/Tutorials:-Virtual-Robot#cameras))

__WARNING__: If you compiled naoqi_driver from source, you first need to source the catkin_workspace of the driver before running qibullet_launcher.py with the same terminal.

Once the qibullet_launcher.py is running, your PyRobot scripts or the one provided in the _examples/pepper_ folder can be launched. You can modify the __qibullet_launcher.py__ script to launch a simulation that suits your needs (subscribing to another camera, changing the environment, running in headless mode, etc.). More information can be found in the [simulator's wiki](https://github.com/softbankrobotics-research/qibullet/wiki)

<p align="middle">
    <img src="https://raw.githubusercontent.com/softbankrobotics-research/qibullet/master/resources/ros_compat.gif" align="middle" width="70%"/>
</p>

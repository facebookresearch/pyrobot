---
id: software
title: Installing the robot software
sidebar_label: Install software
---

**PyRobot 0.0.1** is currently only supported on **Ubuntu 16.04**.

## Installing both PyRobot and LoCoBot

* Install **Ubuntu 16.04**

* Download the installation script

```bash
sudo apt update
sudo apt-get install curl
curl 'https://raw.githubusercontent.com/facebookresearch/pyrobot/master/robots/LoCoBot/install/locobot_install_all.sh' > locobot_install_all.sh
```

* Run the script to install everything (ROS, realsense driver, etc.). 

If you want to use real LoCoBot robot, please run the following command:
**Please connect the nuc machine to a realsense camera before running the following commands**.
  ```bash
  #-t Decides the type of installation. Available Options: full or sim_only
  #-p Decides the python version for pyRobot. Available Options: 2 or 3
  #-l Decides the type of LoCoBot hardware platform. Available Options: cmu or interbotix
  chmod +x locobot_install_all.sh
  ./locobot_install_all.sh -t full -p 2 -l interbotix
  ```

If you want to use simulated LoCoBot in Gazebo only, please run the following commands instead:
  ```bash
  #-t Decides the type of installation. Available Options: full or sim_only
  #-p Decides the python version for pyRobot. Available Options: 2 or 3
  #-l Decides the type of LoCoBot hardware platform. Available Options: cmu or interbotix
  chmod +x locobot_install_all.sh
  ./locobot_install_all.sh -t sim_only -p 2 -l interbotix
  ```

**Note**: To install Python 3 compatible PyRobot, modify ```-p 2``` to ```-p 3``` in the above commands.

## Installing just PyRobot

If you have done the steps above, you don't need to run the steps below.

* Install **Ubuntu 16.04** 

* Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

* Install PyRobot

  ```bash
  cd ~
  mkdir -p low_cost_ws/src
  cd ~/low_cost_ws/src
  source ~/${virtualenv_name}/bin/activate
  git clone --recurse-submodules https://github.com/facebookresearch/pyrobot.git
  cd pyrobot/
  chmod +x install_pyrobot.sh
  ./install_pyrobot.sh -p 2  #For python3, modify the argumet to -p 3 
  ```


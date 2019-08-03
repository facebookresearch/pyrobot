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

* Run the script to install everything (ROS, realsense driver, etc.). **Please connect the NUC machine to a realsense camera before running the following commands**.
```bash
chmod +x locobot_install_all.sh 
./locobot_install_all.sh
```


## Installing just PyRobot

If you have done the steps above, you don't need to run the steps below.

* Install **Ubuntu 16.04** 

* Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

* Install KDL

```bash
sudo apt-get -y install ros-kinetic-orocos-kdl ros-kinetic-kdl-parser-py ros-kinetic-python-orocos-kdl ros-kinetic-trac-ik
```

* Install Python virtual environment

```bash
sudo apt-get -y install python-virtualenv
virtualenv_name="pyenv_pyrobot"
VIRTUALENV_FOLDER=~/${virtualenv_name}
virtualenv --system-site-packages -p python2.7 $VIRTUALENV_FOLDER
```

* Install PyRobot 

```bash
cd ~
mkdir -p low_cost_ws/src
cd ~/low_cost_ws/src
source ~/${virtualenv_name}/bin/activate
git clone --recurse-submodules https://github.com/facebookresearch/pyrobot.git
cd pyrobot
pip install .
```

---
id: next-software
title: Install PyRobot Next Version
sidebar_label: Install Software
---

**PyRobot 0.4** is currently only supported on **Ubuntu 18.04** with **Python 3**.


## Installing PyRobot with Locobot and Franka Panda

* Follow [Franka FCI Documentation](https://frankaemika.github.io/docs/) to setup the machine.

* Download the installation script

```bash
sudo apt update
sudo apt-get install curl
curl 'https://raw.githubusercontent.com/facebookresearch/pyrobot/API_0.4/install_all.sh' > install_all.sh
```
* Run the script to install everything (ROS, realsense driver, etc.).

If you want to use real LoCoBot robot, please run the following command:
**Please connect the nuc machine to a realsense camera before running the following commands**.
  ```bash
  #-t Decides the type of installation. Available Options: full or sim_only
  #-l Decides the type of LoCoBot hardware platform. Available Options: cmu or interbotix
  chmod +x install_all.sh
  ./install_all.sh -t full -l interbotix
  ```

If you want to use simulated LoCoBot in Gazebo only, please run the following commands instead:
  ```bash
  #-t Decides the type of installation. Available Options: full or sim_only
  #-l Decides the type of LoCoBot hardware platform. Available Options: cmu or interbotix
  chmod +x install_all.sh
  ./install_all.sh -t sim_only -l interbotix
  ```

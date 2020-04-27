<a href="https://www.pyrobot.org/"><img class="doc_vid" src="docs/website/website/static/img/pyrobot.svg"></a>

[PyRobot](https://www.pyrobot.org/) is a light weight, high-level interface which provides hardware independent APIs for robotic manipulation and navigation. This repository also contains the low-level stack for [LoCoBot](http://locobot.org), a low cost mobile manipulator hardware platform.

- [What can you do with PyRobot?](#what-can-you-do-with-pyrobot)
- [Installation](#installation)
- [Getting Started](#getting-started)
- [The Team](#the-team)
- [Citation](#citation)
- [License](#license)
- [Future features](#Future-features)

## What can you do with PyRobot?

<p align="center">
    <img src="https://thumbs.gfycat.com/FickleSpeedyChimneyswift-size_restricted.gif", height="180">
    <img src="https://thumbs.gfycat.com/FinishedWeirdCockerspaniel-size_restricted.gif", height="180">
    <img src="https://thumbs.gfycat.com/WeightyLeadingGrub-size_restricted.gif", height="180">
</p>

## Installation

### Installing both PyRobot and LoCoBot dependencies

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

### Installing just PyRobot

* Install **Ubuntu 16.04**

* Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

* Install PyRobot

  ```bash
  cd ~
  mkdir -p low_cost_ws/src
  cd ~/low_cost_ws/src
  git clone --recurse-submodules https://github.com/facebookresearch/pyrobot.git
  cd pyrobot/
  chmod +x install_pyrobot.sh
  ./install_pyrobot.sh -p 2  #For python3, modify the argumet to -p 3 
  ```

**Warning**: As realsense keeps updating, compatibility issues might occur if you accidentally update
realsense-related packages from `Software Updater` in ubuntu. Therefore, we recommend you not to update
any libraries related to realsense. Check the list of updates carefully when ubuntu prompts software udpates.

## Getting Started
Please refer to [pyrobot.org](https://pyrobot.org/) and [locobot.org](http://locobot.org)

## The Team

[Adithya Murali](http://adithyamurali.com/), [Tao Chen](https://taochenshh.github.io), [Dhiraj Gandhi](http://www.cs.cmu.edu/~dgandhi/), Kalyan Vasudev, [Lerrel Pinto](http://www.cs.cmu.edu/~lerrelp/), [Saurabh Gupta](http://saurabhg.web.illinois.edu) and [Abhinav Gupta](http://www.cs.cmu.edu/~abhinavg/). We would also like to thank everyone who has helped PyRobot in any way.

## Future features

We are planning several features, namely:
* Interfacing with other simulators like [AI Habitat](https://aihabitat.org)
* Gravity compensation
* PyRobot interface for [UR5](https://www.universal-robots.com)

## Citation
```
@article{pyrobot2019,
  title={PyRobot: An Open-source Robotics Framework for Research and Benchmarking},
  author={Adithyavairavan Murali and Tao Chen and Kalyan Vasudev Alwala and Dhiraj Gandhi and Lerrel Pinto and Saurabh Gupta and Abhinav Gupta},
  journal={arXiv preprint arXiv:1906.08236},
  year={2019}
}
```
## License
PyRobot is under MIT license, as found in the LICENSE file.

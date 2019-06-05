---
id: crashing
title: Running crash avoidance on your robot
sidebar_label: [Advanced] Avoid crashing
---

## Setup
In this example we will run through basic navigation tools currently available on PyRobot. 

**TODO**: Add a video showing what we will learn here. Currently the videos are all the way at the bottom.

This tutorial can also be run using a simulated version of the robot. Before we get started with this, ensure the following:

* The robot base is supported by PyRobot. Check if your robot is supported [here](gs_overview.md).

* The robot base is switched ON. With the LoCoBot the base beeps when connected.

* The appropriate python virtual environment has been sourced before running any PyRobot package.

<!--DOCUSAURUS_CODE_TABS-->
<!--Sourcing virtual env-->
```bash
source ~/pyenv_pyrobot/bin/activate
```
<!--END_DOCUSAURUS_CODE_TABS-->

* The robot's launch file has been run.

```bash
roslaunch locobot_control main.launch use_base:=true use_arm:=true base:=kobuki use_camera:=true
```

## Running the example

```bash
cd ~/low_cost_ws/src/pyrobot/examples/crash_detection
python locobot_kobuki.py --n_secs=50 --n_loops=1000 --visualize
```

## Acknowledgments

The crash model used is from the [Learning to Fly by Crashing](https://arxiv.org/abs/1704.05588) paper.

```
@inproceedings{gandhi2017learning,
  title={Learning to fly by crashing},
  author={Gandhi, Dhiraj and Pinto, Lerrel and Gupta, Abhinav},
  booktitle={2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={3948--3955},
  year={2017},
  organization={IEEE}
}
```

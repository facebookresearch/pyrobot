---
id: sim2real
title: Running sim2real for arm
sidebar_label: [Advanced] Sim2Real
---
## What is Sim2Real?

In this section we will train a simple RL policy which will learn inverse kinematics for the arm using RL(TD3). The input to the
RL agent is state(joint angles of arm) & goal location(x,y,z) and the control action is the change in each joint angles
to achieve the desired goal.

For TD3 implementation , we used the publicly available [code](https://github.com/sfujim/TD3).

Here is a demo video showing what one can accomplish through this tutorial.

<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/oE6YI4lSCgE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

## Installation

* Activate the virtual environment

```bash
source ~/pyenv_pyrobot/bin/activate
```

* Install Dependencies

```bash
pip install gym
pip install pybullet
```

* Start by either *training the agent from scratch* **or** *downloading pretrained model*

Train the agent (this might take around an hour)
```bash
cd ~/low_cost_ws/src/pyrobot/examples/sim2real
python train.py --exp_name expID01 --valid_goals --save_models
```

Download pretrained model
```bash
cd ~/low_cost_ws/src/pyrobot/examples/sim2real
wget -O log.zip https://www.dropbox.com/s/8cttp55odd7e79o/log.zip?dl=0
unzip log.zip
rm log.zip
```

* Launch the real robot (open a new terminal）

```bash
roslaunch locobot_control main.launch use_arm:=true
```

* Test the above trained policy on the real robot

```bash
source ~/pyenv_pyrobot/bin/activate
cd ~/low_cost_ws/src/pyrobot/examples/sim2real
python test.py --use_real_robot --valid_goals --directory ./log/expID01/pytorch_models --file_name LocoBotEnv-v0_0
```

## Acknowledgments

 The TD3 implementation we have used publicaly available [code](https://github.com/sfujim/TD3) for [Addressing Function Approximation Error in Actor-Critic Methods](https://arxiv.org/abs/1802.09477) paper.

 ```
@article{DBLP:journals/corr/abs-1802-09477,
  author    = {Scott Fujimoto and
               Herke van Hoof and
               Dave Meger},
  title     = {Addressing Function Approximation Error in Actor-Critic Methods},
  journal   = {CoRR},
  volume    = {abs/1802.09477},
  year      = {2018},
  url       = {http://arxiv.org/abs/1802.09477},
  archivePrefix = {arXiv},
  eprint    = {1802.09477},
  timestamp = {Mon, 13 Aug 2018 16:47:21 +0200},
  biburl    = {https://dblp.org/rec/bib/journals/corr/abs-1802-09477},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```

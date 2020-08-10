---
id: grasping
title: Running grasping example on your robot
sidebar_label: [Advanced] Grasping
---

Here is a demo video showing what one can accomplish through this tutorial.
<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/MdDUE-hCL24" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

## Getting started

Here we demonstrate how the PyRobot API can be used to grasp objects by using a learned model. Ideally PyRobot has already been installed, which will allow us to use the Python API for basic robot operations. Before we get started with this, ensure the following:

* The robot arm is supported by PyRobot. Check if your robot is supported [here](gs_overview.md).

* The robot arm is switched ON. With the LoCoBot this is done by connecting the power supply and USB to the arm.

* Setup the virtual environment. Since the grasp models we are using need PyTorch, we need to install it in a new virtual environment. To do this easily, run the following lines.

```bash
virtualenv_name="pyenv_locobot_grasping"
virtualenv --system-site-packages -p python ~/${virtualenv_name}
source ~/${virtualenv_name}/bin/activate
cd ~/low_cost_ws/src/pyrobot
pip install -e .
cd ~/low_cost_ws/src/pyrobot/examples/grasping
pip install -r requirements.txt
```

* The robot's launch file has been run. Note that you have to set `use_arm:=true` and `use_camera:=true`.

```bash
roslaunch locobot_control main.launch use_arm:=true use_camera:=true
```

## Running the example

We use a sampling based grasping algorithm to grasp objects using the LoCoBot. The grasping script `locobot.py` accepts 4 parameters: `n_grasps`, `n_samples`, `patch_size`, and `no_visualize`. `n_grasps` is the number of times the robot attempts a grasp. `n_samples` is the number of samples of size `patch_size` that are input into the grasp model. The larger the number of samples, the more is the inference time. Infering on 100 patches should take around 30 seconds on the NUC. After every grasp inference, a window showing the best found grasp is displayed until the user hits the space key. Running the script with `--no_visualize` disables this visualization.

```bash
source ~/pyenv_locobot_grasping/bin/activate
cd ~/low_cost_ws/src/pyrobot/examples/grasping
python locobot.py --n_grasps=5 --n_samples=100 --patch_size=100
```

## Acknowledgments

The grasp model used is from the [Robot Learning in Homes](http://papers.nips.cc/paper/8123-robot-learning-in-homes-improving-generalization-and-reducing-dataset-bias.pdf) paper.

```
@inproceedings{gupta2018robot,
  title={Robot learning in homes: Improving generalization and reducing dataset bias},
  author={Gupta, Abhinav and Murali, Adithyavairavan and Gandhi, Dhiraj Prakashchand and Pinto, Lerrel},
  booktitle={Advances in Neural Information Processing Systems},
  pages={9112--9122},
  year={2018}
}
```

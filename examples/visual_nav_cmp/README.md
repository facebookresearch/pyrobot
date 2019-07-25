---
id: visual_nav_cmp
title: Cognitive Mapping and Planning
sidebar_label: [Advanced] Visual Navigation (CMP)
---

This example deploys visual navigation policies trained in Cognitive Mapping
and Planning paper [[1](#references)] onto [LoCoBot](http://locobot.org) using
[TensorFlow](https://www.tensorflow.org/). These policies were trained in
simulation on Matterport scans from [[2,3](#references)], and are being run on
the real robot *as is*.  These policies take the pointgoal target, and the
current **RGB image** from the on-board camera as input, and outputs discrete
actions `stay`, `turn left 90 degree`, `turn right 90 degree`, `move forward
40cm`. These actions are implemented on Locobot using ILQR controllers
implemented in `PyRobot` library.  These policies assume a grid world and
perfect odometry. For this example deployment, we assume that odometry from
wheel encoders is perfect. This example is based on code released with
[[1](#references)].

### Setup
This assumes that you have followed instructions for installing `PyRobot` API.
1. Launch Robot in a terminal
    ```bash
    # Launch the robot using the following launch command.
    roslaunch locobot_control main.launch use_base:=true use_camera:=true \
        use_arm:=false use_sim:=false base:=kobuki use_rviz:=false
    ```

2. Install additional dependencies (Open a new terminal)

    ```bash
    source ~/pyenv_pyrobot/bin/activate
    cd ~/low_cost_ws/src/pyrobot/examples/visual_nav_cmp/
    pip install -r requirements.txt
    ```

3. Get pre-trained models

    ```bash
    wget https://www.dropbox.com/s/vw7aqmitsm3kas0/model.tgz?dl=0 -O model.tgz
    tar -xf model.tgz
    ```

### Test Setup
Confirm that tensorflow is properly setup, by running the following command.
```bash
pytest test_cmp.py
```

### Running
```bash
# Run CMP policy using the following command, going forward 1.2m.
python run_cmp.py --goal_x 1.2 --goal_y 0.0 --goal_t 0. --botname locobot
```

### Demo Runs
Videos for some successful demo runs (at 10x speed).
1. Go forward `4m`: `python run_cmp.py --goal_x 4.0 --goal_y 0.0 --goal_t 0. --compensate`.

   <img class="doc_img" src="https://thumbs.gfycat.com/NextEssentialGordonsetter-size_restricted.gif">

2. Go forward `4m`: `python run_cmp.py --goal_x 4.0 --goal_y 0.0 --goal_t 0. --compensate`.

   <img class="doc_img" src="https://thumbs.gfycat.com/FinishedWeirdCockerspaniel-size_restricted.gif">

3. Go forward `2m`, left `2.4m`: `python run_cmp.py --goal_x 2.0 --goal_y 2.4 --goal_t 0. --compensate`.

   <img class="doc_img" src="https://thumbs.gfycat.com/PreciousMajorFeline-size_restricted.gif">

4. Go forward `3.2m`: `python run_cmp.py --goal_x 3.2 --goal_y 0.0 --goal_t 0. --compensate`.

   <img class="doc_img" src="https://thumbs.gfycat.com/SpiffyClassicArabianhorse-size_restricted.gif">


### References
1. [Cognitive Mapping and Planning for Visual
Navigation](https://arxiv.org/pdf/1702.03920.pdf). IJCV 2019. Saurabh Gupta,
Varun Tolani James Davidson, Sergey Levine, Rahul Sukthankar, and Jitendra
Malik.
2. [3D semantic parsing of large-scale indoor spaces](http://buildingparser.stanford.edu/images/3D_Semantic_Parsing.pdf).
CVPR 2016.  Iro Armeni, Ozan Sener, Amir R. Zamir, Helen Jiang, Ioannis
Brilakis, Martin Fischer, Silvio Savarese.
3. [Matterport3D: Learning from RGB-D Data in Indoor Environments](https://arxiv.org/abs/1709.06158). 3DV 2017.
Angel Chang, Angela Dai, Thomas Funkhouser, Maciej Halber, Matthias Niessner, Manolis Savva, Shuran Song, Andy Zeng, Yinda Zhang

### Citing
If you find this policy useful, please consider citing the following paper:
```
@article{gupta2019cognitive,
    author = "Gupta, Saurabh and Tolani, Varun and Davidson, James and Levine, Sergey and Sukthankar, Rahul and Malik, Jitendra",
    title = "Cognitive mapping and planning for visual navigation",
    journal = "International Journal of Computer Vision",
    year = "2019"
}
```

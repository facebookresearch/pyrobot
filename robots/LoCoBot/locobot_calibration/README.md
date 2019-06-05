---
id: calibration
title: Camera Calibration
sidebar_label: [Basic] Camera Calibration
---

## Camera Calibration

### Setup
Calibration depends upon `torch`, `torchvision`, `numpy` and `opencv-python`
packages. These should have been installed when you used the installation script to install `LoCoBot` and `PyRobot`.

### Run Calibration (One time)
1. Start all the ROS services.
```
# Launch the arm controllers, ar_tag tracker, etc. Make sure to specify the
# correct robot base (kobuki or create).
roslaunch locobot_calibration calibrate.launch base:=kobuki
```

2. Run script to collect data for calibration (should take about 10 minutes).
The robot will move the arm and camera around and record images and joint
angles. Keep robot arm workspace clear of obstacles.
```
source ~/pyenv_pyrobot/bin/activate
cd ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_calibration/

# By default the calibration routine will use 6 fixed poses. You can modify
# these poses as necessary in scripts/collect_calibration_data.py
python scripts/collect_calibration_data.py \
    --data_dir tmp/calibration-data/v1/ \
    --botname locobot
```

3. Run script to solve for calibration. 

```
python scripts/solve_for_calibration_params.py \
    --data_dir tmp/calibration-data/v1/ \
    --calibration_output_file ~/.robot/calibrated.json \
    --overwrite --n_iters 10001 --to_optimize camera_link
```

* Calibration solver setups up a least square pixel re-projection error
  between the estimated location of the AR tag corners using the kinematic
  chain, and the detected location of AR tag corners using basic image
  processing. 
* Transforms listed in `scripts/solve_for_calibration_params.py` are
  optimized using gradient descent (using Adam optimizer).
* Calibration parameters are saved to `calibration_output_file`. 
* Note that the solver first optimizes for translation parameters and then
  estimates both translation and rotation. 
* The solver prints out the pixel error. In practice, we have observed a
  pixel error of upto `5` pixels gives reasonable performance. If the solver
  is only able to converge to a larger error:
  1. Verify the data being used for calibration (`data_dir` will contain a
     directory `tag_detect` that will contain the images with the detected tag
     corners, visually scan to make sure they are accurate and don't have too
     many outliers), 
  2. Play around with the solver parameters (learning rate, Adam
     parameters, robust loss parameters, etc.).
  3. Additionally, also optimize for the `arm_base_link` transform. This
     can be done using the `arm_base_link` to `--to_optimize` flag, as:
     `--to_optimize camera_link,arm_base_link`.


### Publish Estimated Transforms

Running the following service will publish the estimated calibration
parameters. These will get incorporated in the ROS transform tree
automatically. This service must be run anytime the camera is used. 
**If you are using `locobot_control/main.launch`, this will be automatically launched and you do not have to run it separately.**
```
roslaunch locobot_calibration calibration_publish.launch
```

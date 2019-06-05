
# Examples for using PyRobot API to control the manipulator


## Getting started

Ideally PyRobot has already been installed or you have run the installation procedures.

## Running the examples

* Connect the power and usb to the arm
* Open a terminal, source the python virtual environment and start the launch file
```bash
source ~/pyenv_pyrobot/bin/activate
roslaunch locobot_control main.launch
```
* Open a new terminal, source the python virtual environment. Run any example script in the folder

**Warning: Robot will move!**

```bash
source ~/pyenv_pyrobot/bin/activate
python joint_space_control.py
```
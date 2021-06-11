## PyRobot Noetic Installation

1. Follow the instruction [here](http://wiki.ros.org/noetic/Installation/Ubuntu) to install `ros-noetic-desktop-full`.
2. Install all the debian packages: 
```
sudo apt-get install libk4a1.4 ros-noetic-moveit tmux ros-noetic-orocos-kdl ros-noetic-kdl-parser-py ros-noetic-trac-ik ros-noetic-pybind11-catkin ros-noetic-rospy-message-converter python-catkin-tools
```
3. Activate a Conda Env and Install GRPC Controller:
```
conda create -n <your-env-name> python=3.8
conda activate <your-env-name>
git clone git@github.com:fair-robotics/fair-robot-envs
conda install -c file://$(eval pwd)/fair_controller_manager/conda/channel \
  -c fair-robotics \
  -c conda-forge \
  fair-controller-manager
```
4. Clone and install noetic-devel branch, under the same conda env:
```
git clone -b noetic-devel https://github.com/facebookresearch/pyrobot.git
pip install -e .
```
5. Extra installation for kdl kinematics and moveit binding (Optional):
```
mkdir -p pyrobot_catkin_ws/src
cd pyrobot_catkin_ws/src
git clone https://github.com/Jekyll1021/kinematics
git clone https://github.com/Jekyll1021/moveit_pybind.git
cd ..
catkin_make --cmake-args -DPYTHON_EXECUTABLE=$(which python) -DPYTHON_INCLUDE_DIR=<path-to-conda>/anaconda3/envs/<your-env-name>/include/python3.8 -DPYTHON_LIBRARY=<path-to-conda>/anaconda3/envs/<your-env-name>/lib/libpython3.8.so
```

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

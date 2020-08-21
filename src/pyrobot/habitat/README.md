# Examples for using PyRobot to Control LoCoBot in Habitat Simulator

__Note__: 

The Habitat PyRobot examples only work with Python3-PyRobot setup.
Arm on LoCoBot is not support in AI-Habitat.

## Getting Started


This tutorials assumes that you have already installed PyRobot and its virtual environment.


Load the PyRobot virtual environment
```bash
source ~/pyenv_pyrobot_python3/bin/activate
```

If using Ubuntu 16.04, please install gcc 7 and CMake latest versions using the following commands,

```bash
pip install cmake # Inside PyRobot Virtual Environment

sudo apt-get install -y software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install g++-7 -y
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-7 
sudo update-alternatives --config gcc
```


Install Habitat Sim using the installation instructions here,
```bash
git clone --branch stable https://github.com/facebookresearch/habitat-sim.git
cd habitat-sim

pip install -r requirements.txt # Inside PyRobot Virtual Environment
sudo apt-get install -y libjpeg-dev libglm-dev libgl1-mesa-glx libegl1-mesa-dev mesa-utils xorg-dev freeglut3-dev

python setup.py install # Inside PyRobot Virtual Environment
# python setup.py install --with-cuda for cuda enabled machines.
```

Detailed Habitat-Sim installation instrucitons are available [here](https://github.com/facebookresearch/habitat-sim#installation)

## Resources 

For more information on Habitat-sim and its capabilities, users are encouraged to go through its dedicated [tutorials](https://aihabitat.org/)

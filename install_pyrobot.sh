#!/usr/bin/env bash

helpFunction()
{
   echo ""
   echo -e "\t-p Decides the python version for pyRobot. Available Options: 2 or 3"
   exit 1 # Exit script after printing help
}

while getopts "p:" opt
do
   case "$opt" in
      p ) PYTHON_VERSION="$OPTARG" ;;
      ? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done

# Print helpFunction in case parameters are empty
if [ -z "$PYTHON_VERSION" ]; then
   echo "Please select a python version";
   helpFunction
fi

if [ $PYTHON_VERSION != "2" ] && [ $PYTHON_VERSION != "3" ]; then
	echo "Invalid Python version type";
   helpFunction
fi

echo "Python $PYTHON_VERSION chosen for pyRobot installation."
sudo apt-get -y install python-virtualenv
sudo apt-get -y install ros-melodic-orocos-kdl ros-melodic-kdl-parser-py ros-melodic-python-orocos-kdl ros-melodic-trac-ik

if [ $PYTHON_VERSION == "2" ]; then
	virtualenv_name="pyenv_pyrobot_python2"
	VIRTUALENV_FOLDER=~/${virtualenv_name}
	if [ ! -d "$VIRTUALENV_FOLDER" ]; then
		virtualenv --system-site-packages -p python2.7 $VIRTUALENV_FOLDER
		source ~/${virtualenv_name}/bin/activate
		pip install .
		deactivate
		echo "alias load_pyrobot_env='source $VIRTUALENV_FOLDER/bin/activate '" >> ~/.bashrc
	fi
fi

if [ $PYTHON_VERSION == "3" ]; then
	# Make a virtual env to install other dependencies (with pip)
	virtualenv_name="pyenv_pyrobot_python3"
	VIRTUALENV_FOLDER=~/${virtualenv_name}
	if [ ! -d "$VIRTUALENV_FOLDER" ]; then
		sudo apt-get -y install software-properties-common 
		sudo apt-get -y install python-catkin-tools python3.6-dev python3-catkin-pkg-modules python3-numpy python3-yaml python3-rospkg-modules python3-empy
		sudo apt-get -y install python3-tk
		sudo apt-get -y install ros-melodic-orocos-kdl ros-melodic-kdl-parser-py ros-melodic-python-orocos-kdl ros-melodic-trac-ik
		virtualenv -p /usr/bin/python3.6 $VIRTUALENV_FOLDER
		source ~/${virtualenv_name}/bin/activate
		pip install catkin_pkg pyyaml empy rospkg
		python -m pip install --upgrade numpy
		pip install .
		deactivate
	fi

	source ~/${virtualenv_name}/bin/activate
	echo "Setting up PyRobot Catkin Ws..."
	PYROBOT_PYTHON3_WS=~/pyrobot_catkin_ws

	if [ ! -d "$PYROBOT_PYTHON3_WS/src" ]; then
		mkdir -p $PYROBOT_PYTHON3_WS/src

		cd $PYROBOT_PYTHON3_WS/src

		git clone https://github.com/ros/geometry

		git clone https://github.com/ros/geometry2

		# Clone cv_bridge src
		git clone -b python3_patch_melodic https://github.com/kalyanvasudev/vision_opencv.git

		#ros_comm TODO: Remove this when the pull request gets approved
		git clone -b patch-1 https://github.com/kalyanvasudev/ros_comm.git
		
		cd ..
		
		rosdep install --from-paths src --ignore-src -y -r
		
		# Install all the python 3 dependencies
		sudo apt-get -y install ros-melodic-cv-bridge

		# Build
		catkin_make --cmake-args -DPYTHON_EXECUTABLE=$(which python) -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
		
		echo "alias load_pyrobot_env='source $VIRTUALENV_FOLDER/bin/activate && source $PYROBOT_PYTHON3_WS/devel/setup.bash'" >> ~/.bashrc
	fi
	deactivate
fi

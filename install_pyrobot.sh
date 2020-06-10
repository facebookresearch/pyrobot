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

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version == "16.04" ]; then
	echo "Ubuntu 16.04 detected. ROS-Kinetic chosen for installation.";
	ROS_NAME="kinetic"
elif [ $ubuntu_version == "18.04" ]; then
	echo "Ubuntu 18.04 detected. ROS-Melodic chosen for installation.";
	ROS_NAME="melodic"
else
	echo -e "Unsupported Ubuntu verison: $ubuntu_version"
	echo -e "pyRobot only works with 16.04 or 18.04"
	exit 1
fi


echo "Python $PYTHON_VERSION chosen for pyRobot installation."
sudo apt-get -y install python-virtualenv
sudo apt-get -y install ros-$ROS_NAME-orocos-kdl ros-$ROS_NAME-kdl-parser-py ros-$ROS_NAME-python-orocos-kdl ros-$ROS_NAME-trac-ik

if [ $PYTHON_VERSION == "2" ]; then
	virtualenv_name="pyenv_pyrobot_python2"
	VIRTUALENV_FOLDER=~/${virtualenv_name}
	if [ ! -d "$VIRTUALENV_FOLDER" ]; then
		virtualenv --system-site-packages -p python2.7 $VIRTUALENV_FOLDER
		source ~/${virtualenv_name}/bin/activate
		#pip install -U setuptools
		pip install --upgrade 'setuptools<45.0.0'
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
		if [ $ubuntu_version == "16.04" ]; then
			sudo apt-get -y install python-software-properties
			sudo add-apt-repository -y ppa:fkrull/deadsnakes
			#sudo add-apt-repository ppa:jonathonf/python-3.6
		fi
		sudo apt-get update
		sudo apt-get -y install python-catkin-tools python3.6-dev python3-catkin-pkg-modules python3-numpy python3-yaml
		sudo apt-get -y install python3-tk python3.6-tk
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

		if [ $ROS_NAME == "kinetic" ]; then
			git clone -b indigo-devel https://github.com/ros/geometry
			git clone -b indigo-devel https://github.com/ros/geometry2
			git clone -b python3_patch https://github.com/kalyanvasudev/vision_opencv.git
		else
			git clone -b melodic-devel https://github.com/ros/geometry
			git clone -b melodic-devel https://github.com/ros/geometry2
			git clone -b python3_patch_melodic https://github.com/kalyanvasudev/vision_opencv.git
		fi
		
		git clone -b patch-1 https://github.com/kalyanvasudev/ros_comm.git

		cd ..
		
		# Install all the python 3 dependencies
		sudo apt-get -y install ros-$ROS_NAME-cv-bridge

		# Build
		catkin_make --cmake-args -DPYTHON_EXECUTABLE=$(which python) -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
		
		echo "alias load_pyrobot_env='source $VIRTUALENV_FOLDER/bin/activate && source $PYROBOT_PYTHON3_WS/devel/setup.bash'" >> ~/.bashrc
	fi
	deactivate
fi

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
sudo apt-get -y install ros-kinetic-orocos-kdl ros-kinetic-kdl-parser-py ros-kinetic-python-orocos-kdl ros-kinetic-trac-ik

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
		sudo apt-get install software-properties-common python-software-properties
		sudo add-apt-repository ppa:fkrull/deadsnakes
		#sudo add-apt-repository ppa:jonathonf/python-3.6
		sudo apt-get update
		sudo apt-get install python-catkin-tools python3.6-dev python3-catkin-pkg-modules python3-numpy python3-yaml
		sudo apt-get install python3-tk
		sudo apt-get -y install ros-kinetic-orocos-kdl ros-kinetic-kdl-parser-py ros-kinetic-python-orocos-kdl ros-kinetic-trac-ik
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

		#clone tf
		git clone https://github.com/ros/geometry
		git clone -b indigo-devel https://github.com/ros/geometry2
		
		# Clone cv_bridge src
		git clone https://github.com/ros-perception/vision_opencv.git

		#ros_comm TODO: Remove this when the pull request gets approved
		git clone -b patch-1 https://github.com/kalyanvasudev/ros_comm.git
		
		cd ..
		
		# Install all the python 3 dependencies
		sudo apt-get install ros-kinetic-cv-bridge

		## Find version of cv_bridge in your repository
		##apt-cache show ros-kinetic-cv-bridge | grep Version
		 
		# Checkout right version in git repo. In our case it is 1.12.8
		cd src/vision_opencv/
		git checkout 1.12.8 # Usually this is the version!
		cd ../../
		
		#symlink - https://github.com/ros-perception/vision_opencv/issues/196
		my_link=/usr/lib/x86_64-linux-gnu/libboost_python3.so
		if [ ! -L ${my_link} ] ; then
		   sudo ln -s /usr/lib/x86_64-linux-gnu/libboost_python-py35.so /usr/lib/x86_64-linux-gnu/libboost_python3.so
		fi
		# Build
		catkin_make
		
		echo "alias load_pyrobot_env='source $VIRTUALENV_FOLDER/bin/activate && source $PYROBOT_PYTHON3_WS/devel/setup.bash'" >> ~/.bashrc
	fi
	deactivate
fi
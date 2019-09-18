#!/usr/bin/env bash

helpFunction()
{
   echo ""
   echo -e "\t-t Decides the type of installation. Available Options: full or sim_only"
   echo -e "\t-p Decides the python version for pyRobot. Available Options: 2 or 3"
   exit 1 # Exit script after printing help
}

while getopts "t:p:" opt
do
   case "$opt" in
      t ) INSTALL_TYPE="$OPTARG" ;;
      p ) PYTHON_VERSION="$OPTARG" ;;
      ? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done

# Print helpFunction in case parameters are empty
if [ -z "$INSTALL_TYPE" ] || [ -z "$PYTHON_VERSION" ]; then
   echo "Some or all of the parameters are empty";
   helpFunction
fi

# Check if the parameters are valid
if [ $INSTALL_TYPE != "full" ] && [ $INSTALL_TYPE != "sim_only" ]; then
	echo "Invalid Installation type";
   helpFunction
fi

if [ $PYTHON_VERSION != "2" ] && [ $PYTHON_VERSION != "3" ]; then
	echo "Invalid Python version type";
   helpFunction
fi

echo "$INSTALL_TYPE installation type is chosen for LoCoBot."
echo "Python $PYTHON_VERSION chosen for pyRobot installation."

trap "exit" INT TERM ERR
trap "kill 0" EXIT
echo -e "\e[1;33m ******************************************* \e[0m"
echo -e "\e[1;33m The installation takes around half an hour! \e[0m"
echo -e "\e[1;33m ******************************************* \e[0m"
sleep 4
start_time="$(date -u +%s)"

install_packages () {
	pkg_names=("$@")
	for package_name in "${pkg_names[@]}"; 
	do
		if [ $(dpkg-query -W -f='${Status}' $package_name 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
		    sudo apt-get -y install $package_name
		else
		    echo "${package_name} is already installed";
		fi
	done
}


# STEP 0 - Make sure you have installed Ubuntu 16.04, and upgrade to lastest dist
if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then 
        sudo apt-get update && sudo apt-get -y upgrade && sudo apt-get -y dist-upgrade
fi


# STEP 1 - Install basic dependencies
declare -a package_names=(
	"vim" 
	"git" 
	"terminator"
	"python-pip"
	"python-dev"
	"python-virtualenv"
	"screen"
	"openssh-server" 
	"libssl-dev" 
	"libusb-1.0-0-dev"
	"libgtk-3-dev" 
	"libglfw3-dev"
	)
install_packages "${package_names[@]}"

sudo pip install --upgrade cryptography
sudo python -m easy_install --upgrade pyOpenSSL
sudo pip install --upgrade pip


# STEP 2 - Install ROS Kinetic
if [ $(dpkg-query -W -f='${Status}' ros-kinetic-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then 
	echo "Installing ROS..."
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt-get update
	sudo apt-get -y install ros-kinetic-desktop-full
	if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
	    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
	fi
	sudo rosdep init
	rosdep update
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
else
	echo "ros-kinetic-desktop-full is already installed";
fi
source /opt/ros/kinetic/setup.bash


# STEP 3 - Install ROS debian dependencies
declare -a ros_package_names=(
	"ros-kinetic-dynamixel-motor" 
	"ros-kinetic-moveit" 
	"ros-kinetic-trac-ik"
	"ros-kinetic-ar-track-alvar"
	"ros-kinetic-turtlebot-*"
	"ros-kinetic-move-base"
	"ros-kinetic-ros-control"
	"ros-kinetic-gazebo-ros-control"
	"ros-kinetic-ros-controllers"
	"ros-kinetic-navigation"
	"ros-kinetic-rgbd-launch"
	"ros-kinetic-kdl-parser-py"
	"ros-kinetic-orocos-kdl"
	"ros-kinetic-python-orocos-kdl"
	"ros-kinetic-turtlebot"
	)

install_packages "${ros_package_names[@]}"

if [ $INSTALL_TYPE == "full" ]; then

	# STEP 4 - Install camera (Intel Realsense D435)
	echo "Installing camera dependencies..."

	# STEP 4A: Install librealsense
	if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
		sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
		sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
		sudo apt-get update
		version="2.18.1-0~realsense0.568"
		sudo apt-get -y install librealsense2-udev-rules=${version}
		sudo apt-get -y install librealsense2-dkms=1.3.4-0ubuntu1
		sudo apt-get -y install librealsense2=${version}
		sudo apt-get -y install librealsense2-utils=${version}
		sudo apt-get -y install librealsense2-dev=${version}
		sudo apt-get -y install librealsense2-dbg=${version}
	fi

	# STEP 4B: Install realsense2 SDK from source (in a separate catkin workspace)
	CAMERA_FOLDER=~/camera_ws
	if [ ! -d "$CAMERA_FOLDER/src" ]; then
		mkdir -p $CAMERA_FOLDER/src
		cd $CAMERA_FOLDER/src/
		catkin_init_workspace
	fi
	if [ ! -d "$CAMERA_FOLDER/src/realsense" ]; then
		cd $CAMERA_FOLDER/src/
		git clone https://github.com/intel-ros/realsense.git
		cd realsense
		git checkout a036d81bcc6890658104a8de1cba24538effd6e3
	fi
	if [ -d "$CAMERA_FOLDER/devel" ]; then
		rm -rf $CAMERA_FOLDER/devel
	fi
	if [ -d "$CAMERA_FOLDER/build" ]; then
		rm -rf $CAMERA_FOLDER/build
	fi
	cd $CAMERA_FOLDER
	catkin_make clean
	catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
	catkin_make install
	echo "source ~/camera_ws/devel/setup.bash" >> ~/.bashrc
	source ~/camera_ws/devel/setup.bash
fi

# STEP 5 - Setup catkin workspace
echo "Setting up robot software..."
LOCOBOT_FOLDER=~/low_cost_ws
if [ ! -d "$LOCOBOT_FOLDER/src" ]; then
	mkdir -p $LOCOBOT_FOLDER/src
	cd $LOCOBOT_FOLDER/src
	catkin_init_workspace
fi
if [ ! -d "$LOCOBOT_FOLDER/src/pyrobot" ]; then
	cd $LOCOBOT_FOLDER/src
	git clone --recurse-submodules https://github.com/facebookresearch/pyrobot.git
	if [ $PYTHON_VERSION == "3" ]; then
		cd pyrobot
		git checkout python3
	fi
fi
cd $LOCOBOT_FOLDER
rosdep update 
rosdep install --from-paths src -i -y
cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/install
chmod +x install_orb_slam2.sh
source install_orb_slam2.sh
cd $LOCOBOT_FOLDER
if [ -d "$LOCOBOT_FOLDER/devel" ]; then
	rm -rf $LOCOBOT_FOLDER/devel
fi
if [ -d "$LOCOBOT_FOLDER/build" ]; then
	rm -rf $LOCOBOT_FOLDER/build
fi
catkin_make
echo "source $LOCOBOT_FOLDER/devel/setup.bash" >> ~/.bashrc
source $LOCOBOT_FOLDER/devel/setup.bash


if [ $INSTALL_TYPE == "full" ]; then
	# STEP 6 - Dependencies and config for calibration
	chmod +x src/pyrobot/robots/LoCoBot/locobot_navigation/orb_slam2_ros/scripts/gen_cfg.py
	rosrun orb_slam2_ros gen_cfg.py
	HIDDEN_FOLDER=~/.robot
	if [ ! -d "$HIDDEN_FOLDER" ]; then
		mkdir ~/.robot
		cp $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_calibration/config/default.json ~/.robot/
	fi
fi

# STEP 7 - Make a virtual env to install other dependencies (with pip)
if [ $PYTHON_VERSION == "2" ]; then
	virtualenv_name="pyenv_pyrobot"
	VIRTUALENV_FOLDER=~/${virtualenv_name}
	if [ ! -d "$VIRTUALENV_FOLDER" ]; then
		virtualenv --system-site-packages -p python2.7 $VIRTUALENV_FOLDER
		source ~/${virtualenv_name}/bin/activate
		cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot
		pip install --ignore-installed -r requirements.txt
		cd $LOCOBOT_FOLDER/src/pyrobot/
		pip install .
		deactivate
	fi
fi
if [ $PYTHON_VERSION == "3" ]; then
	cd $LOCOBOT_FOLDER/src/pyrobot
	chmod +x install_pyrobot.sh
	source install_pyrobot.sh
fi

# STEP 8 - Setup udev rules
cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot
sudo cp thirdparty/udev_rules/*.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
sudo udevadm trigger
sudo usermod -a -G dialout $USER

end_time="$(date -u +%s)"

elapsed="$(($end_time-$start_time))"

echo "Installation complete, took $elapsed seconds in total"
echo "NOTE: Remember to logout and login back again before using the robot!"
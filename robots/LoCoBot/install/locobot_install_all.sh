#!/usr/bin/env bash

helpFunction()
{
   echo ""
   echo -e "\t-t Decides the type of installation. Available Options: full or sim_only"
   echo -e "\t-p Decides the python version for pyRobot. Available Options: 2 or 3"
   echo -e "\t-l Decides the type of LoCoBot hardware platform. Available Options: cmu or interbotix"
   exit 1 # Exit script after printing help
}

while getopts "t:p:l:" opt
do
   case "$opt" in
      t ) INSTALL_TYPE="$OPTARG" ;;
      p ) PYTHON_VERSION="$OPTARG" ;;
      l ) LOCOBOT_PLATFORM="$OPTARG" ;;
      ? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done

# Print helpFunction in case parameters are empty
if [ -z "$INSTALL_TYPE" ] || [ -z "$PYTHON_VERSION" ] || [ -z "$LOCOBOT_PLATFORM" ]; then
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

if [ $LOCOBOT_PLATFORM != "cmu" ] && [ $LOCOBOT_PLATFORM != "interbotix" ]; then
	echo "Invalid LoCoBot hardware platform type";
   helpFunction
fi

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version == "16.04" ]; then
	ROS_NAME="kinetic"
elif [ $ubuntu_version == "18.04" ]; then
	ROS_NAME="melodic"
else
	echo -e "Unsupported Ubuntu verison: $ubuntu_version"
	echo -e "pyRobot only works with 16.04 or 18.04"
	exit 1
fi

echo "Ubuntu $ubuntu_version detected. ROS-$ROS_NAME chosen for installation.";

echo "$INSTALL_TYPE installation type is chosen for LoCoBot."
echo "Python $PYTHON_VERSION chosen for pyRobot installation."
echo "$LOCOBOT_PLATFORM hardware platform chosen for LoCoBot."

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
#if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then 
#        sudo apt-get update && sudo apt-get -y upgrade && sudo apt-get -y dist-upgrade
#fi


# STEP 1 - Install basic dependencies
declare -a package_names=(
	"python-tk"
	"python-sip"
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
sudo pip install --upgrade pip==20.3


# STEP 2 - Install ROS 

if [ $ROS_NAME == "kinetic" ]; then

	if [ $(dpkg-query -W -f='${Status}' ros-kinetic-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then 
		echo "Installing ROS..."
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros1-latest.list'
		sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
		sudo apt-get update
		sudo apt-get -y install ros-kinetic-desktop-full
		if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
		    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
		fi
		sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
		sudo apt -y install python-rosdep
		sudo rosdep init
		rosdep update
		echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	else
		echo "ros-kinetic-desktop-full is already installed";
	fi
else
	if [ $(dpkg-query -W -f='${Status}' ros-melodic-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then 
		echo "Installing ROS..."
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list'
		sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
		sudo apt-get update
		sudo apt-get -y install ros-melodic-desktop-full
		if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
			sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
		fi
		sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
		sudo apt -y install python-rosdep
		sudo rosdep init
		rosdep update
		echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	else
		echo "ros-melodic-desktop-full is already installed";
	fi
fi

source /opt/ros/$ROS_NAME/setup.bash


# STEP 3 - Install ROS debian dependencies
declare -a ros_package_names=(
	"ros-$ROS_NAME-dynamixel-motor" 
	"ros-$ROS_NAME-moveit" 
	"ros-$ROS_NAME-trac-ik"
	"ros-$ROS_NAME-ar-track-alvar"
	"ros-$ROS_NAME-move-base"
	"ros-$ROS_NAME-ros-control"
	"ros-$ROS_NAME-gazebo-ros-control"
	"ros-$ROS_NAME-ros-controllers"
	"ros-$ROS_NAME-navigation"
	"ros-$ROS_NAME-rgbd-launch"
	"ros-$ROS_NAME-kdl-parser-py"
	"ros-$ROS_NAME-orocos-kdl"
	"ros-$ROS_NAME-python-orocos-kdl"
  	"ros-$ROS_NAME-ddynamic-reconfigure"
	#"ros-$ROS_NAME-libcreate"
	)

install_packages "${ros_package_names[@]}"

if [ $INSTALL_TYPE == "full" ]; then

	# STEP 4 - Install camera (Intel Realsense D435)
	echo "Installing camera dependencies..."

	# STEP 4A: Install librealsense
	if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
		sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
		sudo apt-get update
		version="2.33.1-0~realsense0.2140"
		sudo apt-get -y install librealsense2-udev-rules=${version}
		sudo apt-get -y install librealsense2-dkms=1.3.11-0ubuntu1
		sudo apt-get -y install librealsense2=${version}
    		sudo apt-get -y install librealsense2-gl=${version}
		sudo apt-get -y install librealsense2-utils=${version}
		sudo apt-get -y install librealsense2-dev=${version}
		sudo apt-get -y install librealsense2-dbg=${version}
		sudo apt-mark hold librealsense2*
	fi

	# STEP 4B: Install realsense2 SDK from source (in a separate catkin workspace)
	CAMERA_FOLDER=~/camera_ws
	if [ ! -d "$CAMERA_FOLDER/src" ]; then
		mkdir -p $CAMERA_FOLDER/src
		cd $CAMERA_FOLDER/src/
		catkin_init_workspace
	fi
	if [ ! -d "$CAMERA_FOLDER/src/realsense-ros" ]; then
		cd $CAMERA_FOLDER/src/
    		git clone https://github.com/IntelRealSense/realsense-ros.git
		cd realsense-ros/
		git checkout 2.2.13
	fi
	if [ -d "$CAMERA_FOLDER/devel" ]; then
		rm -rf $CAMERA_FOLDER/devel
	fi
	if [ -d "$CAMERA_FOLDER/build" ]; then
		rm -rf $CAMERA_FOLDER/build
	fi
        if [ -d "$CAMERA_FOLDER/install" ]; then
	        rm -rf $CAMERA_FOLDER/install
        fi
	cd $CAMERA_FOLDER
	catkin_make clean
	catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
	catkin_make install
	echo "source $CAMERA_FOLDER/devel/setup.bash" >> ~/.bashrc
	source $CAMERA_FOLDER/devel/setup.bash
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
	git clone https://github.com/facebookresearch/pyrobot.git
	cd pyrobot
	git checkout master
	git submodule update --init --recursive
  if [ $LOCOBOT_PLATFORM == "cmu" ]; then
    cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_description/urdf
    ln cmu_locobot_description.urdf locobot_description.urdf
    cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_moveit_config/config
    ln cmu_locobot.srdf locobot.srdf
    cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_control/src
    sed -i 's/\(float restJnts\[5\] = \)\(.*\)/\1{0, -0.3890, 1.617, -0.1812, 0.0153};/' locobot_controller.cpp
  fi
  if [ $LOCOBOT_PLATFORM == "interbotix" ]; then
    cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_description/urdf
    ln interbotix_locobot_description.urdf locobot_description.urdf
    cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_moveit_config/config
    ln interbotix_locobot.srdf locobot.srdf
    cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_control/src
    sed -i 's/\(float restJnts\[5\] = \)\(.*\)/\1{0, -1.30, 1.617, 0.5, 0};/' locobot_controller.cpp
  fi
fi

if [ ! -d "$LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/thirdparty" ]; then

  	cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot
  	mkdir thirdparty
  	cd thirdparty
		git clone https://github.com/AutonomyLab/create_autonomy
		git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
		git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
		git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
		git clone https://github.com/ros-controls/ros_control.git
		git clone https://github.com/kalyanvasudev/ORB_SLAM2.git
		git clone https://github.com/s-gupta/ar_track_alvar.git

	if [ $ROS_NAME == "kinetic" ]; then
		cd create_autonomy && git checkout 90e597ea4d85cde1ec32a1d43ea2dd0b4cbf481c && cd ..
		cd dynamixel-workbench && git checkout bf60cf8f17e8385f623cbe72236938b5950d3b56 && cd ..
		cd DynamixelSDK && git checkout 05dcc5c551598b4d323bf1fb4b9d1ee03ad1dfd9 && cd ..
		cd dynamixel-workbench-msgs && git checkout 93856f5d3926e4d7a63055c04a3671872799cc86 && cd ..
		cd ros_control && git checkout 44cf68aab6cb1293e91f69ef7efe30b80195356b && cd ..
		cd ORB_SLAM2 && git checkout ec8d750d3fc813fe5cef82f16d5cc11ddfc7bb3d && cd ..
		cd ar_track_alvar && git checkout 625a3cf928552ee5bf97453897af6790d523962f && cd ..
	else
		cd create_autonomy && git checkout 90e597ea4d85cde1ec32a1d43ea2dd0b4cbf481c && cd ..
		cd dynamixel-workbench && git checkout bf60cf8f17e8385f623cbe72236938b5950d3b56 && cd ..
		cd DynamixelSDK && git checkout 05dcc5c551598b4d323bf1fb4b9d1ee03ad1dfd9 && cd ..
		cd dynamixel-workbench-msgs && git checkout 93856f5d3926e4d7a63055c04a3671872799cc86 && cd ..
		cd ros_control && git checkout cd39acfdb2d08dc218d04ff98856b0e6a525e702 && cd ..
		cd ORB_SLAM2 && git checkout ec8d750d3fc813fe5cef82f16d5cc11ddfc7bb3d && cd ..
		cd ar_track_alvar && git checkout a870d5f00a548acb346bfcc89d42b997771d71a3 && cd ..
	fi
fi

cd $LOCOBOT_FOLDER
rosdep update 
rosdep install --from-paths src/pyrobot -i -y
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

if [ ! -d "$LOCOBOT_FOLDER/src/turtlebot" ]; then
	cd $LOCOBOT_FOLDER/src/
	mkdir turtlebot
	cd turtlebot

	git clone https://github.com/turtlebot/turtlebot_simulator
	git clone https://github.com/turtlebot/turtlebot.git
	git clone https://github.com/turtlebot/turtlebot_apps.git
	git clone https://github.com/turtlebot/turtlebot_msgs.git
	git clone https://github.com/turtlebot/turtlebot_interactions.git

	git clone https://github.com/toeklk/orocos-bayesian-filtering.git
	cd orocos-bayesian-filtering/orocos_bfl/
	./configure
	make
	sudo make install
	cd ../
	make
	cd ../

	git clone https://github.com/udacity/robot_pose_ekf
	git clone https://github.com/ros-perception/depthimage_to_laserscan.git

	git clone https://github.com/yujinrobot/kobuki_msgs.git
	git clone https://github.com/yujinrobot/kobuki_desktop.git
	cd kobuki_desktop/
	rm -r kobuki_qtestsuite
	cd -
	git clone https://github.com/yujinrobot/kobuki.git
	cd kobuki && git checkout $ROS_NAME && cd ..
	mv kobuki/kobuki_description kobuki/kobuki_bumper2pc \
	  kobuki/kobuki_node kobuki/kobuki_keyop \
	  kobuki/kobuki_safety_controller ./
	
	#rm -r kobuki

	git clone https://github.com/yujinrobot/yujin_ocs.git
	mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers .
	mv yujin_ocs/yocs_safety_controller yujin_ocs/yocs_velocity_smoother .
	rm -rf yujin_ocs

	sudo apt-get install ros-$ROS_NAME-kobuki-* -y
	sudo apt-get install ros-$ROS_NAME-ecl-streams -y
fi

# STEP 6 - Make a virtual env to install other dependencies (with pip)
if [ $PYTHON_VERSION == "2" ]; then
	cd $LOCOBOT_FOLDER/src/pyrobot
	chmod +x install_pyrobot.sh
	source install_pyrobot.sh  -p 2
	
	virtualenv_name="pyenv_pyrobot_python2"
	source ~/${virtualenv_name}/bin/activate
	cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot
	pip install --ignore-installed -r requirements_python2.txt
	
	cd $LOCOBOT_FOLDER
	source /opt/ros/$ROS_NAME/setup.bash
 	if [ $INSTALL_TYPE == "full" ]; then
    	        source $CAMERA_FOLDER/devel/setup.bash
  	fi
	pip install catkin_pkg pyyaml empy rospkg
	catkin_make
	echo "source $LOCOBOT_FOLDER/devel/setup.bash" >> ~/.bashrc
	source $LOCOBOT_FOLDER/devel/setup.bash
	deactivate
fi
if [ $PYTHON_VERSION == "3" ]; then
	cd $LOCOBOT_FOLDER
	source /opt/ros/$ROS_NAME/setup.bash
  	if [ $INSTALL_TYPE == "full" ]; then
    	        source $CAMERA_FOLDER/devel/setup.bash
  	fi
	pip install catkin_pkg pyyaml empy rospkg
	catkin_make
	echo "source $LOCOBOT_FOLDER/devel/setup.bash" >> ~/.bashrc
	source $LOCOBOT_FOLDER/devel/setup.bash
	
	cd $LOCOBOT_FOLDER/src/pyrobot
	chmod +x install_pyrobot.sh
	source install_pyrobot.sh  -p 3

	virtualenv_name="pyenv_pyrobot_python3"
	cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot
	source ~/${virtualenv_name}/bin/activate
	pip3 install --ignore-installed -r requirements_python3.txt
	deactivate
fi


if [ $INSTALL_TYPE == "full" ]; then
	# STEP 7 - Dependencies and config for calibration
	cd $LOCOBOT_FOLDER
	chmod +x src/pyrobot/robots/LoCoBot/locobot_navigation/orb_slam2_ros/scripts/gen_cfg.py
	rosrun orb_slam2_ros gen_cfg.py
	HIDDEN_FOLDER=~/.robot
	if [ ! -d "$HIDDEN_FOLDER" ]; then
		mkdir ~/.robot
		cp $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_calibration/config/default.json ~/.robot/
	fi
	
	# STEP 8 - Setup udev rules
	cd $LOCOBOT_FOLDER/src/pyrobot/robots/LoCoBot
	sudo cp udev_rules/*.rules /etc/udev/rules.d
	sudo service udev reload
	sudo service udev restart
	sudo udevadm trigger
	sudo usermod -a -G dialout $USER
fi


end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"

echo "Installation complete, took $elapsed seconds in total"
echo "NOTE: Remember to logout and login back again before using the robot!"

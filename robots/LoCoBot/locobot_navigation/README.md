## Installation Instructions for Intel RealSense

The following insturctions work for **ROS Kinetic (Ubuntu 16.04)**, the official installation tutorial is [here](https://github.com/intel-ros/realsense)
### Step 1: Install Intel RealSense SDK 2.0
```bash
#sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get update
version="2.16.1-0~realsense0.88"
sudo apt-get install librealsense2-dkms=1.3.1-0ubuntu3
sudo apt-get install librealsense2=${version}
sudo apt-get install librealsense2-utils=${version}  
sudo apt-get install librealsense2-dev=${version}
sudo apt-get install librealsense2-dbg=${version}
```
**If you want to remove all the realsense sdk-related packages**:
```bash
dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge
```
### Step 2: Install Intel RealSense ROS from sources
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
catkin_init_workspace
git clone https://github.com/intel-ros/realsense.git
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install rgbd_launch
```bash
sudo apt-get install ros-kinetic-rgbd-launch
```

### Step 4: Test the camera node
To view the rgb and depth images only:
```bash
roslaunch realsense2_camera rs_camera.launch
```
To view the pointcloud:
```bash
roslaunch realsense2_camera rs_rgbd.launch
```
Then you can use **rviz** to view the topics.

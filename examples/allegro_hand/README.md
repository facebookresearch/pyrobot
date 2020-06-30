# Examples for using PyRobot API to control Allegro Hand

This tutorials assumes that you have installed PyRobot already using the official instructions.

## Note
This tutorial has been tested to work with the following setup,
1. [Allegro hand V4.0](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_v4.0)
2. [PEAk PCAN-USB FD adaptor](https://www.peak-system.com/PCAN-USB-FD.365.0.html?&L=1)
3. Ubuntu 16.04 with Kernel 4.15
4. [Peak pcan driver 8.9.3](http://www.peak-system.com/fileadmin/media/linux/version-history.html)
5. ROS-Kinetic
6. Python3.6

## Setting up the hardware
Please wire the Allgero hand and the PCAN-USB FD adator using these (insturctions)[http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Wiring_for_Allegro_Hand_v4.0]
More instructions on trouble shooting the hardware are available here. (http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_v4.0)

## Setting up the Allegro Hand software

1. Intall these packages
```bash
sudo apt-get install libpopt-dev ros-kinetic-libpcan
```

2. Download the peak pcan driver v8.9.3 from (here)(http://www.peak-system.com/fileadmin/media/linux/version-history.html)

Install the drivers:
```bash
make clean; make NET=NO_NETDEV_SUPPORT
sudo make install
sudo /sbin/modprobe pcan
```

Test that the interface is installed properly with:
```bash
 cat /proc/pcan
```
You should see some stuff streaming.

When the hnd is connected to the computer using the PCA-USB FD adaptor, you should see pcanusb0 or pcanusb1 or pcanusb32 in the list of avaiable interfaces:
```bash
ls -l /dev/pcan*
```

3. Setup the Allegro hand ros packages,
```bash
mkdir -p ~/allegro_hand_ws/src # feel free to change the directory location to your convenience
cd ~/allegro_hand_ws/src
git clonehttps://github.com/kalyanvasudev/allegro_hand_ros_v4.git
cd ~/allegro_had_ws
catkin_make
```


##Running the examples

1st terminal,
```bash
source ~/allegro_had_ws/devel/setup.bash
roslaunch roslaunch allegro_hand allegro_hand.launch HAND:=left AUTO_CAN:=false CAN_DEVICE:=/dev/pcan32 JSP_GUI:=false CONTROLLER:=grasp 
# Note that CAN_DEVICE should be set as per the output of ls -l /dev/pcan*
```
2nd terminal,
```bash
load_pyenv_pyrobot 
python allegro_hand_example.py
```


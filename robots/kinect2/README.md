# Exmaples for using PyRobot to use Kinect2

## Getting started

Make sure your system has [IAI Kinect2](https://github.com/code-iai/iai_kinect2) installed. If not, please follow the [install steps](https://github.com/code-iai/iai_kinect2#install) to setup KinectV2.

## Getting started
1. Launch Kinect2 camera
```bash
cd ~/catkin_ws # or the appropriate catkin workspace in which IAI Kinect2 package is in
source devel~/setuo.bash
roslaunch kinect2_bridge kinect2_bridge.launch
```

2. Run Pyrobot examples in a new terminal
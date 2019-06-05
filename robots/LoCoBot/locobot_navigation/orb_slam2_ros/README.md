## Setting
```bash
cd orb_slam2_ros
echo "export ORBSLAM2_LIBRARY_PATH=${ORB_SLAM2_PATH}/ORB_SLAM2" >> ~/.bashrc
source ~/.bashrc
cd bash_scripts
chmod +x launch_kinect2.sh
```

## Run
```bash
rosnode cleanup 
roslaunch orb_slam2_ros lcr_realsense.launch
```

If you get errors like `Failed to load nodelet '/camera/realsense2_camera` or `An exception has been thrown: Time is out of dual 32-bit range`, one possible reason is that the realsense driver node is not killed properly previously and you can use `rosnode list` to see the nodes that are running. You may see that even though you killed the camera driver's launch file with `ctrl+c`, the nodes (`/camera/realsense2_camera, /camera/realsense2_camera_manager`) are still running. So you need to call `rosnode cleanup` to kill the nodes and then you will be able to launch the camera driver again.

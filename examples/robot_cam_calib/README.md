# Robot Calibration
**NOTE:** Currently steps are mainly for Kinect

**TODO:** Need to add other cameras as well

1. Launch the robot 
2. Launch the camera 
```
roslaunch kinect2_bridge kinect2_bridge.launch
```
3. Launch the ar tag detector (make sure you provide proper ar-tag size in cm)

For Kinect2
```
roslaunch ar_track_alvar pr2_indiv_no_kinect.launch marker_size:=10.0 cam_image_topic:=/kinect2/hd/image_color cam_info_topic:=/kinect2/hd/camera_info output_frame:=kinect2_rgb_optical_frame
```


4. Record the data

Attach ArMarker to Robot gripper and note the id. There are 2 ways to record data points

* Manual
![Auto data collection Demo](imgs/manual_data_collection.gif)

In this you will move the arm and record tha data points
```buildoutcfg
python collect_data_manual.py --ar_id 1
```

* Auto
![Auto data collection Demo](imgs/auto_data_collection.gif)

**NOTE:** Make sure you add the obstacles in moveit according to your environment before running this
```buildoutcfg
python collect_data_auto.py --ar_id 1 --x_range 0.1 0.2 --y_range -0.1 0.1 --z_range 0.5 0.6 --quat 0.0 0.0 0.0 1.0
```
**NOTE:** Change the ar_id based on the id of ar marker you are using

4. Run the calibration script on the data
```buildoutcfg
python calib_robot_cam.py --vis
```

5. Publish Base-Camera Transform

Run the command mentioned as output in step 4

6. Check Calibration
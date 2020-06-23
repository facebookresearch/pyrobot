---
id: faq
title: Frequently Asked Questions
sidebar_label: FAQ
---

## Frequently Asked Questions

**1. What should I do when I get error messages in main.launch?**

Here are some common errors:
* "There is no status packet! Can't find Dynamixel ID or BaudRate"
	* Check if you have connected the power or battery cable to the arm [Dynamixel HUB](https://locobot-website.netlify.com/docs/k_step_2). Ensure that all the motor cables are properly inserted. Check to see if one of the motor LEDs is blinking red and read on below.
* "No transform available between frame 'odom' and planning frame '/base_footprint'"
	* Check if you have turned on the Kobuki base or if it has sufficient battery
* "No RealSense devices were found! Terminating RealSense Node..."
	* Check if you have connected the USB 3 camera cable to the camera. If that does not fix it, please read on below, it could be a firmware issue.
* For other errors, please post a Github issue

**2. My create2 base is not connecting properly?**

While most create2 bases on LoCoBot-lite work fine, we have noticed that some bases fail to connect properly (even when they are fully powered). If we find a resolution, we will update it here. In the mean time, we recommend new users to assemble the standard LoCoBot with the kobuki base.

**3. What should I do when my arm motor shuts down or a motor LED is blinking red?**

**TL;DR** Power cycling the arm will reset the motors. Simply disconnect and reconnect the power cable to the [Dynamixel HUB](https://locobot-website.netlify.com/docs/k_step_2) mounted on the base of the LoCoBot.

When any one of the arm's dynamixel motors shutsdown, the entire robot controller will shutdown for safety reasons. When this happens, the LED on the shutdown motor will be blinking red and more information will be recorded in the log of *main.launch*. The most common issue we have seen is a overload error, when the motor operates beyond its payload capacity or if the arm hits an obstacle that it is unable to move past to its desired location. Tuning the controller gains or making the arm lift a heavy payload can also result in this error.

**4. I'm not able to get the rgb image or depth image from PyRobot (it returns None)**

There are several possiblities. 
* Check if the camera connection is good.
* You might need to update the realsense firmware. Instructions can be found below.

<!--DOCUSAURUS_CODE_TABS-->
<!--Update realsense firmware-->
```bash
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
sudo apt-get update
sudo apt-get install intel-realsense-dfu*
cd ~/Downloads
wget https://downloadmirror.intel.com/28573/eng/D400_Series_Development_FW_5_11_4.zip
unzip D400_Series_Development_FW_5_11_4.zip
lsusb #find out the bus and device numbers for realsense camera
# use the bus and device numbers in the following command (e.g. -b 002 -d 003)
intel-realsense-dfu -b <bus number> -d <device number> -f -i Signed_Image_UVC_5_11_4_0.bin
```
<!--END_DOCUSAURUS_CODE_TABS--> 

* You might need to install the latest realsense driver. Instructions can be found [here](https://github.com/IntelRealSense/realsense-ros).
* If you cannot get the point cloud via `LoCoBotCamera.get_current_pcd`, check if ORB-SLAM2 is running and if it's tracking properly. If the ORB-SLAM2 is lost (because of too few keypoints in the frame) in the first place, point cloud might not be generated.


**5. Got timeout error (`timeout exceeded while waiting for service /move_base/GlobalPlanner/make_plan`) when using base?**

* Check if the base is turned on.
* Check if the base and the computer is well connected.

**6. How do I integrate a new robot into PyRobot?**

This is a great idea! Please see the steps in [New Robot Support](https://www.pyrobot.org/docs/new_robot_support).

**7. If I modify the LoCoBot hardware, how much of the software can I still use?**

For most cases, you should be able to re-use almost all the PyRobot and LoCoBot code. Please make a new Github issue (with a question label) to discuss specific details. If your new robot setup is sufficiently different, it is better to declare it as a new robot with its own config file (see [New Robot Support](https://www.pyrobot.org/docs/new_robot_support))

**8. Gripper fingers get stuck when closing and opening**

This is probably because of friction and over use. Applying some lubricant will fix this issue.

**9. How do I contribute?**

Thanks in advance for your contributions. Please see [contribute.md](https://github.com/facebookresearch/pyrobot/blob/master/CONTRIBUTING.md)

**10. PyRobot doesn't work well if Robot is launched with different settings?**

If you kill and launch the robot again, you need to import PyRobot in a new python terminal.

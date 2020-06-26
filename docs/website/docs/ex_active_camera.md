---
id: active_camera
title: Running camera tools on your robot
sidebar_label: [Basic] Active Camera
---

Here is a demo video showing what one can accomplish through this tutorial.

<figure class="video_container">
  <iframe class="doc_vid" src="https://www.youtube.com/embed/M6QxNV3dfV4" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

## Getting started

* Make sure robot is turned ON

* Launch the robot driver

<!--DOCUSAURUS_CODE_TABS-->
<!--Launch driver-->
```bash
roslaunch locobot_control main.launch use_camera:=true
```
<!--END_DOCUSAURUS_CODE_TABS--> 

* Start the python virtual environment

<!--DOCUSAURUS_CODE_TABS-->
<!--Source virtual env-->
```bash
load_pyrobot_env
```
<!--END_DOCUSAURUS_CODE_TABS--> 

## Example usage

* Initialize a robot instance
<!--DOCUSAURUS_CODE_TABS-->
<!--Create robot-->
```py
from pyrobot import Robot

bot = Robot('locobot')

bot.camera.reset()
```
<!--END_DOCUSAURUS_CODE_TABS--> 

* Set camera pan or tilt angle, or set pan and tilt angles together
<!--DOCUSAURUS_CODE_TABS-->
<!--Set Pan-->
```py
pan = 0.4
bot.camera.set_pan(pan, wait=True)
```
<!--Set Tilt-->
```py
tilt = 0.4
bot.camera.set_tilt(tilt, wait=True)
``` 
<!--Set Pan and Tilt-->
```py
camera_pose = [0, 0.7]
bot.camera.set_pan_tilt(camera_pose[0], camera_pose[1], wait=True)
```
<!--END_DOCUSAURUS_CODE_TABS--> 

* Get rgb or depth images
<!--DOCUSAURUS_CODE_TABS-->
<!--RGB-->
```py
from pyrobot.utils.util import try_cv2_import
cv2 = try_cv2_import()

rgb = bot.camera.get_rgb()
cv2.imshow('Color', rgb[:, :, ::-1])
cv2.waitKey(3000)
```
<!--Depth-->
```py
from pyrobot.utils.util import try_cv2_import
cv2 = try_cv2_import()

import numpy as np
depth = bot.camera.get_depth()
actual_depth_values = depth.astype(np.float64) / 1000.0
cv2.imshow('Depth', depth)
cv2.waitKey(3000)
``` 
<!--RGB and Depth-->
```py
from pyrobot.utils.util import try_cv2_import
cv2 = try_cv2_import()

rgb, depth = bot.camera.get_rgb_depth()
cv2.imshow('Color', rgb[:, :, ::-1])
cv2.imshow('Depth', depth)
cv2.waitKey(3000)
```
<!--END_DOCUSAURUS_CODE_TABS--> 

* Get 3D point coordinates of pixels in RGB image

In manipulation, we often get into the case where we want to know where an RGB pixel is in the 3D world, meaning the correspondence between RGB pixels and their 3D locations with respect to the robot. In PyRobot, we have provided a utility function (`pix_to_3dpt`) to do this transformation. This function takes as input the row indices and column indices of the pixels. It will return the 3D point locations and the RGB color values of these pixels. You can specify `in_cam=True` to get the 3D locations in the camera frame. Otherwise, the 3D locations will be with respect to the **LoCoBot base frame**.
<!--DOCUSAURUS_CODE_TABS-->
<!--Pixel via Scalar-->
```py
bot.camera.set_tilt(0.6)
# row number in the RGB image
r = 295
# column number in the RGB image
c = 307
pt, color = bot.camera.pix_to_3dpt(r,c)
print('3D point:', pt)
print('Color:', color)
rgb = bot.camera.get_rgb()
cv2.imshow('Color', rgb[:, :, ::-1])
cv2.waitKey(10000)
```
<!--Pixels via List-->
```py
bot.camera.set_tilt(0.6)
r = [295, 360]
c = [307, 296]
# this will return the point and color of 
# pixel (295, 307) and pixel (360, 296)
pt, color = bot.camera.pix_to_3dpt(r,c)
print('3D point:', pt)
print('Color:', color)
rgb = bot.camera.get_rgb()
cv2.imshow('Color', rgb[:, :, ::-1])
cv2.waitKey(10000)
```
<!--Pixels via Numpy Array-->
```py
import numpy as np
bot.camera.set_tilt(0.6)
r = np.array([295, 360])
c = np.array([307, 296])
# this will return the point and color of 
# pixel (295, 307) and pixel (360, 296)
pt, color = bot.camera.pix_to_3dpt(r,c)
print('3D point:', pt)
print('Color:', color)
rgb = bot.camera.get_rgb()
cv2.imshow('Color', rgb[:, :, ::-1])
cv2.waitKey(10000)
```
<!--END_DOCUSAURUS_CODE_TABS--> 

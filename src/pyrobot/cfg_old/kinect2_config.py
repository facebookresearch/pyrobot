# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from pyrobot.cfg.config import get_cfg_defaults

_C = get_cfg_defaults()

_C.HAS_ARM = False
_C.HAS_BASE = False
_C.HAS_GRIPPER = False

_CAMERAC = _C.CAMERA
# CAMERA class name
_CAMERAC.CLASS = "Kinect2Camera"
# topic name of the camera info
_CAMERAC.ROSTOPIC_CAMERA_INFO_STREAM = "/kinect2/qhd/camera_info"
# TOD0: Make sure the topic names are right
# topic name of the RGB images
_CAMERAC.ROSTOPIC_CAMERA_RGB_STREAM = "/kinect2/qhd/image_color_rect"
# topic name of the depth images
_CAMERAC.ROSTOPIC_CAMERA_DEPTH_STREAM = "/kinect2/qhd/image_depth_rect"
# depth map factor
_CAMERAC.DEPTH_MAP_FACTOR = 1000.0


def get_cfg():
    return _C.clone()

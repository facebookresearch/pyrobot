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
_CAMERAC.CLASS = "AzureKinectCamera"
# topic name of the camera info
_CAMERAC.ROSTOPIC_CAMERA_INFO_STREAM = "/rgb/camera_info"
# TOD0: Make sure the topic names are right
# topic name of the RGB images
_CAMERAC.ROSTOPIC_CAMERA_RGB_STREAM = "/rgb/image_raw"
# topic name of the depth images
_CAMERAC.ROSTOPIC_CAMERA_DEPTH_STREAM = "/depth_to_rgb/image_raw"
# depth map factor
_CAMERAC.DEPTH_MAP_FACTOR = 1.0


def get_cfg():
    return _C.clone()

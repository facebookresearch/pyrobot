# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import rospkg
import threading
import yaml
from copy import deepcopy

import message_filters
import numpy as np
import pyrobot.utils.util as prutil
import rospy

from pyrobot.core import Camera
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError


class Kinect2Camera(Camera):
    """
    This is camera class that interfaces with the KinectV2 camera
    """

    def __init__(self, configs, ns=""):
        """
        Constructor of the KinectV2Camera class.

        :param configs: Camera specific configuration object

        :type configs: YACS CfgNode
        """
        super(Kinect2Camera, self).__init__(configs=configs, ns=ns)

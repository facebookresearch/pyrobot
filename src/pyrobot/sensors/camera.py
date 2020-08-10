
from __future__ import print_function

import copy
import importlib
import os
import sys
import threading
import time
from abc import ABCMeta, abstractmethod

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import JointState, CameraInfo, Image

import pyrobot.utils.util as prutil

from pyrobot.utils.move_group_interface import MoveGroupInterface as MoveGroup
from pyrobot_bridge.srv import *

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError

import message_filters

import actionlib
from pyrobot_bridge.msg import (
    MoveitAction,
    MoveitGoal,
)
from actionlib_msgs.msg import GoalStatus

class Camera(object):
    """
    This is a parent class on which the robot
    specific Camera classes would be built.
    """

    __metaclass__ = ABCMeta

    def __init__(self, configs):
        """
        Constructor for Camera parent class.

        :param configs: configurations for camera
        :type configs: YACS CfgNode
        """
        self.configs = configs
        self.cv_bridge = CvBridge()
        self.camera_info_lock = threading.RLock()
        self.camera_img_lock = threading.RLock()
        self.rgb_img = None
        self.depth_img = None
        self.camera_info = None
        self.camera_P = None
        rospy.Subscriber(
            self.configs.ROSTOPIC_CAMERA_INFO_STREAM,
            CameraInfo,
            self._camera_info_callback,
        )

        rgb_topic = self.configs.ROSTOPIC_CAMERA_RGB_STREAM
        self.rgb_sub = message_filters.Subscriber(rgb_topic, Image)
        depth_topic = self.configs.ROSTOPIC_CAMERA_DEPTH_STREAM
        self.depth_sub = message_filters.Subscriber(depth_topic, Image)
        img_subs = [self.rgb_sub, self.depth_sub]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            img_subs, queue_size=10, slop=0.2
        )
        self.sync.registerCallback(self._sync_callback)

    def _sync_callback(self, rgb, depth):
        self.camera_img_lock.acquire()
        try:
            self.rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")
            self.rgb_img = self.rgb_img[:, :, ::-1]
            self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.camera_img_lock.release()

    def _camera_info_callback(self, msg):
        self.camera_info_lock.acquire()
        self.camera_info = msg
        self.camera_P = np.array(msg.P).reshape((3, 4))
        self.camera_info_lock.release()

    def get_rgb(self):
        """
        This function returns the RGB image perceived by the camera.

        :rtype: np.ndarray or None
        """
        self.camera_img_lock.acquire()
        rgb = copy.deepcopy(self.rgb_img)
        self.camera_img_lock.release()
        return rgb

    def get_depth(self):
        """
        This function returns the depth image perceived by the camera.

        :rtype: np.ndarray or None
        """
        self.camera_img_lock.acquire()
        depth = copy.deepcopy(self.depth_img)
        self.camera_img_lock.release()
        depth = depth / self.configs.CAMERA.DEPTH_MAP_FACTOR
        return depth

    def get_rgb_depth(self):
        """
        This function returns both the RGB and depth
        images perceived by the camera.

        :rtype: np.ndarray or None
        """
        self.camera_img_lock.acquire()
        rgb = copy.deepcopy(self.rgb_img)
        depth = copy.deepcopy(self.depth_img)
        self.camera_img_lock.release()
        return rgb, depth

    def get_intrinsics(self):
        """
        This function returns the camera intrinsics.

        :rtype: np.ndarray
        """
        if self.camera_P is None:
            return self.camera_P
        self.camera_info_lock.acquire()
        P = copy.deepcopy(self.camera_P)
        self.camera_info_lock.release()
        return P[:3, :3]



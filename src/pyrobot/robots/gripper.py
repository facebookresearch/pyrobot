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

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError

import message_filters

import actionlib

from actionlib_msgs.msg import GoalStatus


class Gripper(object):
    """
    This is a parent class on which the robot
    specific Gripper classes would be built.
    """

    __metaclass__ = ABCMeta

    def __init__(self, configs, ns=""):
        """
        Constructor for Gripper parent class.

        :param configs: configurations for gripper
        :type configs: YACS CfgNode
        """
        self.configs = configs
        self.ns = ns

    @abstractmethod
    def open(self, **kwargs):
        pass

    @abstractmethod
    def close(self, **kwargs):
        pass

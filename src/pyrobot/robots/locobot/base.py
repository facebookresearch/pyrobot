# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import copy
import os
from functools import partial
from os.path import expanduser

import numpy as np
import rospy
import tf
import tf.transformations
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from nav_msgs.msg import Odometry

try:
    from orb_slam2_ros.vslam import VisualSLAM

    USE_ORB_SLAM2 = True
except:
    USE_ORB_SLAM2 = False

from pyrobot.robots.base import Base
from std_msgs.msg import Empty

from pyrobot.robots.locobot.base_control_utils import (
    LocalActionStatus,
    LocalActionServer,
)

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from std_msgs.msg import Empty

from actionlib_msgs.msg import GoalStatus


class BaseSafetyCallbacks(object):
    """
    This class encapsulates and provides interface to bumper, cliff and
    wheeldrop sensors on the base.  It all also trigers necessary callbacks
    when these sensors get tripped.
    """

    def __init__(self, base):
        self.should_stop = False
        self.bumper = False
        self.cliff = False
        self.wheel_drop = False
        self.subscribers = []

        s = rospy.Subscriber(
            self.configs.ROSTOPIC_BUMPER,
            BumperEvent,
            self.bumper_callback_kobuki,
        )
        self.subscribers.append(s)
        s = rospy.Subscriber(
            self.configs.ROSTOPIC_CLIFF, CliffEvent, self.cliff_callback
        )
        self.subscribers.append(s)
        s = rospy.Subscriber(
            self.configs.ROSTOPIC_WHEELDROP,
            WheelDropEvent,
            self.wheeldrop_callback,
        )
        self.subscribers.append(s)

    def bumper_callback_create(self, data):
        bumped = data.is_left_pressed or data.is_right_pressed
        to_bump = (
            data.is_light_left
            or data.is_light_center_left
            or data.is_light_center_right
            or data.is_light_front_right
            or data.is_light_right
        )
        if bumped or to_bump:
            if self.bumper is False:
                rospy.loginfo("Bumper Detected")
            self.should_stop = True
            self.bumper = True
        else:
            self.bumper = False

    def cliff_callback(self, data):
        rospy.loginfo("Cliff Detected")
        self.should_stop = True
        self.cliff = True

    def wheeldrop_callback(self, data):
        rospy.loginfo("Drop the contact")
        self.should_stop = True
        self.wheel_drop = True

    def bumper_callback_kobuki(self, date):
        rospy.loginfo("Bumper Detected")
        self.bumper = True
        self.should_stop = True

    def __del__(self):
        # Delete callback on deletion of object.
        for s in self.subscribers:
            s.unregister()

class BaseState(BaseSafetyCallbacks):
    def __init__(self, base, configs):
        # Set up SLAM, if requested.
        self.configs = configs
        BaseSafetyCallbacks.__init__(self, base)

    def __del__(self):
        BaseSafetyCallbacks.__del__(self)


class LoCoBotBase(Base):
    """
    This is a common base class for the locobot and locobot-lite base.
    """

    def __init__(
        self,
        configs,
    ):
        """
        The constructor for LoCoBotBase class.

        :param configs: configurations read from config file

        :type configs: YACS CfgNode
        """
        super(LoCoBotBase, self).__init__(configs=configs)
        use_base = rospy.get_param("use_base", False) or rospy.get_param(
            "use_sim", False
        )
        if not use_base:
            rospy.logwarn(
                "Neither use_base, nor use_sim, is not set to True "
                "when the LoCoBot driver is launched. "
                "You may not be able to command the base "
                "correctly using PyRobot!"
            )
            return

        base = configs.BASE_TYPE
        assert base in [
            "kobuki",
            "create",
        ], "BASE should be one of kobuki, create but is {:s}".format(base)

        self.base_state = BaseState(base, configs)

        rospy.on_shutdown(self.clean_shutdown)

    def clean_shutdown(self):
        rospy.loginfo("Stopping LoCoBot Base. Waiting for base thread to finish")
        self.stop()

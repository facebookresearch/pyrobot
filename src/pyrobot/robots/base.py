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

from pyrobot.utils.util import try_cv2_import, append_namespace

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError

import message_filters

import actionlib

from actionlib_msgs.msg import GoalStatus


class Base(object):
    """
    This is a parent class on which the robot
    specific Base classes would be built.
    """

    def __init__(self, configs, ns=""):
        """
        The consturctor for Base class.

        :param configs: configurations for base
        :type configs: YACS CfgNode
        """
        self.configs = configs
        self.ns = ns
        self.ctrl_pub = rospy.Publisher(
            append_namespace(self.ns, configs.ROSTOPIC_BASE_COMMAND), 
            Twist, 
            queue_size=1
        )

    def stop(self):
        """
        Stop the base
        """
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.ctrl_pub.publish(msg)

    def set_vel(self, fwd_speed, turn_speed, exe_time=1):
        """
        Set the moving velocity of the base

        :param fwd_speed: forward speed
        :param turn_speed: turning speed
        :param exe_time: execution time
        """
        fwd_speed = min(fwd_speed, self.configs.MAX_ABS_FWD_SPEED)
        fwd_speed = max(fwd_speed, -self.configs.MAX_ABS_FWD_SPEED)
        turn_speed = min(turn_speed, self.configs.MAX_ABS_TURN_SPEED)
        turn_speed = max(turn_speed, -self.configs.MAX_ABS_TURN_SPEED)

        msg = Twist()
        msg.linear.x = fwd_speed
        msg.angular.z = turn_speed

        start_time = rospy.get_time()
        self.ctrl_pub.publish(msg)
        while rospy.get_time() - start_time < exe_time:
            self.ctrl_pub.publish(msg)
            rospy.sleep(1.0 / self.configs.BASE_CONTROL_RATE)

    def go_to_relative(self, xyt_position, use_map, close_loop, smooth):
        """
        Moves the robot to the robot to given
        goal state relative to its initial pose.

        :param xyt_position: The  relative goal state of the form (x,y,t)
        :param use_map: When set to "True", ensures that controler is
                        using only free space on the map to move the robot.
        :param close_loop: When set to "True", ensures that controler is
                           operating in open loop by
                           taking account of odometry.
        :param smooth: When set to "True", ensures that the motion
                       leading to the goal is a smooth one.

        :type xyt_position: list
        :type use_map: bool
        :type close_loop: bool
        :type smooth: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        raise NotImplementedError

    def go_to_absolute(self, xyt_position, use_map, close_loop, smooth):
        """
        Moves the robot to the robot to given goal state in the world frame.

        :param xyt_position: The goal state of the form (x,y,t)
                             in the world (map) frame.
        :param use_map: When set to "True", ensures that controler is using
                        only free space on the map to move the robot.
        :param close_loop: When set to "True", ensures that controler is
                           operating in open loop by
                           taking account of odometry.
        :param smooth: When set to "True", ensures that the motion
                       leading to the goal is a smooth one.

        :type xyt_position: list
        :type use_map: bool
        :type close_loop: bool
        :type smooth: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        raise NotImplementedError

    def track_trajectory(self, states, controls, close_loop):
        """
        State trajectory that the robot should track.

        :param states: sequence of (x,y,t) states that the robot should track.
        :param controls: optionally specify control sequence as well.
        :param close_loop: whether to close loop on the
                           computed control sequence or not.

        :type states: list
        :type controls: list
        :type close_loop: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        raise NotImplementedError

    def get_state(self, state_type):
        """
        Returns the requested base pose in the (x,y, yaw) format.

        :param state_type: Requested state type. Ex: Odom, SLAM, etc
        :type state_type: string
        :return: pose: pose of the form [x, y, yaw]
        :rtype: list
        """
        raise NotImplementedError

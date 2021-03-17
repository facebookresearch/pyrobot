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
from .arm import Arm

from pyrobot.utils.util import try_cv2_import, append_namespace

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError

import message_filters

import actionlib

from actionlib_msgs.msg import GoalStatus


class ROSArm(Arm):
    """
    This is a parent class on which the robot
    specific Arm classes would be built.
    """

    def __init__(self, configs, ns=""):
        """
        Constructor for Arm parent class.

        :param configs: configurations for arm
        """
        self.configs = configs
        self.ns = ns

        self.joint_state_lock = threading.RLock()

        self.arm_joint_names = self.configs.JOINT_NAMES
        self.arm_dof = len(self.arm_joint_names)

        # Subscribers
        self._joint_angles = dict()
        self._joint_velocities = dict()
        self._joint_efforts = dict()

        for name in self.arm_joint_names:
            self._joint_angles[name] = 0.
            self._joint_velocities[name] = 0.
            self._joint_efforts[name] = 0.

        rospy.Subscriber(
            append_namespace(self.ns, configs.ROSTOPIC_JOINT_STATES),
            JointState,
            self._callback_joint_states,
        )

        rospy.sleep(1)

        # Publishers
        self.joint_pub = None
        self._setup_joint_pub()

    def get_joint_angles(self):
        """
        Return the joint angles

        :return: joint_angles
        :rtype: np.ndarray
        """
        rospy.sleep(0.1)
        self.joint_state_lock.acquire()
        joint_angles = []
        for joint in self.arm_joint_names:
            joint_angles.append(self.get_joint_angle(joint))
        joint_angles = np.array(joint_angles).flatten()
        self.joint_state_lock.release()
        return joint_angles

    def get_joint_velocities(self):
        """
        Return the joint velocities

        :return: joint_vels
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        joint_vels = []
        for joint in self.arm_joint_names:
            joint_vels.append(self.get_joint_velocity(joint))
        joint_vels = np.array(joint_vels).flatten()
        self.joint_state_lock.release()
        return joint_vels

    def get_joint_torques(self):
        """
        Return the joint torques

        :return: joint_torques
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        joint_torques = []
        for joint in self.arm_joint_names:
            try:
                joint_torques.append(self.get_joint_torque(joint))
            except (ValueError, IndexError):
                rospy.loginfo("Torque value for joint " "[%s] not available!" % joint)
        joint_torques = np.array(joint_torques).flatten()
        self.joint_state_lock.release()
        return joint_torques

    def get_joint_angle(self, joint):
        """
        Return the joint angle of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint angle
        :rtype: float
        """
        if joint not in self.arm_joint_names:
            raise ValueError("%s not in arm joint list!" % joint)
        if joint not in self._joint_angles.keys():
            raise ValueError("Joint angle for joint $s not available!" % joint)
        return self._joint_angles[joint]

    def get_joint_velocity(self, joint):
        """
        Return the joint velocity of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint velocity
        :rtype: float
        """
        if joint not in self.arm_joint_names:
            raise ValueError("%s not in arm joint list!" % joint)
        if joint not in self._joint_velocities.keys():
            raise ValueError("Joint velocity for joint" " $s not available!" % joint)
        return self._joint_velocities[joint]

    def get_joint_torque(self, joint):
        """
        Return the joint torque of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint torque
        :rtype: float
        """
        if joint not in self.arm_joint_names:
            raise ValueError("%s not in arm joint list!" % joint)
        if joint not in self._joint_efforts.keys():
            raise ValueError("Joint torque for joint $s" " not available!" % joint)
        return self._joint_efforts[joint]

    def set_joint_positions(self, positions, wait=True, **kwargs):
        """
        Sets the desired joint angles for all arm joints

        :param positions: list of length #of joints, angles in radians
        :param plan: whether to use moveit to plan a path. Without planning,
                     there is no collision checking and each joint will
                     move to its target joint position directly.
        :param wait: if True, it will wait until the desired
                     joint positions are achieved
        :type positions: list
        :type plan: bool
        :type wait: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """

        self._pub_joint_positions(positions)
        if wait:
            result = self._loop_angle_pub_cmd(self._pub_joint_positions, positions)
            return result

    def set_joint_velocities(self, velocities, **kwargs):
        """
        Sets the desired joint velocities for all arm joints

        :param velocities: target joint velocities
        :type velocities: list
        """
        self._pub_joint_velocities(velocities)

    def set_joint_torques(self, torques, **kwargs):
        """
        Sets the desired joint torques for all arm joints

        :param torques: target joint torques
        :type torques: list
        """
        self._pub_joint_torques(torques)

    def _callback_joint_states(self, msg):
        """
        ROS subscriber callback for arm joint state (position, velocity)

        :param msg: Contains message published in topic
        :type msg: sensor_msgs/JointState
        """
        self.joint_state_lock.acquire()
        for idx, name in enumerate(msg.name):
            if name in self.arm_joint_names:
                if idx < len(msg.position):
                    self._joint_angles[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._joint_velocities[name] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self._joint_efforts[name] = msg.effort[idx]
        self.joint_state_lock.release()

    def _pub_joint_positions(self, positions):
        joint_state = JointState()
        joint_state.position = tuple(positions)
        self.joint_pub.publish(joint_state)

    def _pub_joint_velocities(self, velocities):
        joint_state = JointState()
        joint_state.velocity = tuple(velocities)
        self.joint_pub.publish(joint_state)

    def _pub_joint_torques(self, torques):
        joint_state = JointState()
        joint_state.effort = tuple(torques)
        self.joint_pub.publish(joint_state)

    def _angle_error_is_small(self, target_joints):
        cur_joint_vals = self.get_joint_angles()
        joint_diff = cur_joint_vals - np.array(target_joints)
        error = np.max(np.abs(joint_diff))
        if error <= self.configs.MAX_JOINT_ERROR:
            return joint_diff, error, True
        else:
            return joint_diff, error, False

    def _loop_angle_pub_cmd(self, cmd, joint_vals):
        start_time = time.time()
        vel_zero_times = 0
        # wait 10s in worse case
        success = False
        for i in range(int(10 / self.configs.WAIT_MIN_TIME)):
            cmd(joint_vals)
            res = self._angle_error_is_small(joint_vals)
            joint_diff, error, eflag = res
            if eflag:
                success = True
                break
            vels = self.get_joint_velocities()
            es_time = time.time() - start_time
            if es_time > 0.5 and np.max(np.abs(vels)) < 0.01:
                vel_zero_times += 1
            else:
                vel_zero_times = 0
            if vel_zero_times > 10:
                success = False
                break
            rospy.sleep(self.configs.WAIT_MIN_TIME)
        return success

    def _setup_joint_pub(self):
        self.joint_pub = rospy.Publisher(
            append_namespace(self.ns, self.configs.ROSTOPIC_SET_JOINT),
            JointState,
            queue_size=1,
        )

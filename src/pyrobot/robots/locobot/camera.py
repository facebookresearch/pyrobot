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

from pyrobot.sensors.camera import Camera
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf import TransformListener

from pyrobot.utils.util import try_cv2_import, append_namespace

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError


def constrain_within_range(value, MIN, MAX):
    return min(max(value, MIN), MAX)


def is_within_range(value, MIN, MAX):
    return (value <= MAX) and (value >= MIN)


class LoCoBotCamera(Camera):
    """
    This is camera class that interfaces with the Realsense
    camera and the pan and tilt joints on the robot.
    """

    def __init__(self, configs, ns=""):
        """
        Constructor of the LoCoBotCamera class.

        :param configs: Object containing configurations for camera,
                        pan joint and tilt joint.

        :type configs: YACS CfgNode
        """
        use_camera = rospy.get_param("use_camera", False)
        use_sim = rospy.get_param("use_sim", False)
        use_camera = use_camera or use_sim
        if not use_camera:
            rospy.logwarn(
                "Neither use_camera, nor use_sim, is not set"
                " to True when the LoCoBot driver is launched."
                "You may not be able to command the camera"
                " correctly using PyRobot!!!"
            )
            return
        super(LoCoBotCamera, self).__init__(configs=configs, ns=ns)

        rospy.Subscriber(
            append_namespace(self.ns, self.configs.ROSTOPIC_JOINT_STATES),
            JointState,
            self._camera_pose_callback,
        )

        self.set_pan_pub = rospy.Publisher(
            append_namespace(self.ns, self.configs.ROSTOPIC_SET_PAN),
            Float64,
            queue_size=1,
        )
        self.set_tilt_pub = rospy.Publisher(
            append_namespace(self.ns, self.configs.ROSTOPIC_SET_TILT),
            Float64,
            queue_size=1,
        )

        self.pan = None
        self.tilt = None
        self.tol = 0.01

    def _camera_pose_callback(self, msg):
        if "head_pan_joint" in msg.name:
            pan_id = msg.name.index("head_pan_joint")
            self.pan = msg.position[pan_id]
        if "head_tilt_joint" in msg.name:
            tilt_id = msg.name.index("head_tilt_joint")
            self.tilt = msg.position[tilt_id]

    @property
    def state(self):
        """
        Return the current pan and tilt joint angles of the robot camera.

        :return:
                pan_tilt: A list the form [pan angle, tilt angle]
        :rtype: list
        """
        return self.get_state()

    def get_state(self):
        """
        Return the current pan and tilt joint angles of the robot camera.

        :return:
                pan_tilt: A list the form [pan angle, tilt angle]
        :rtype: list
        """
        return [self.pan, self.tilt]

    def get_pan(self):
        """
        Return the current pan joint angle of the robot camera.

        :return:
                pan: Pan joint angle
        :rtype: float
        """
        return self.pan

    def get_tilt(self):
        """
        Return the current tilt joint angle of the robot camera.

        :return:
                tilt: Tilt joint angle
        :rtype: float
        """
        return self.tilt

    def set_pan(self, pan, wait=True):
        """
        Sets the pan joint angle to the specified value.

        :param pan: value to be set for pan joint
        :param wait: wait until the pan angle is set to
                     the target angle.

        :type pan: float
        :type wait: bool
        """
        pan = constrain_within_range(
            np.mod(pan + np.pi, 2 * np.pi) - np.pi,
            self.configs.MIN_PAN,
            self.configs.MAX_PAN,
        )
        self.set_pan_pub.publish(pan)
        if wait:
            for i in range(30):
                rospy.sleep(0.1)
                if np.fabs(self.get_pan() - pan) < self.tol:
                    break

    def set_tilt(self, tilt, wait=True):
        """
        Sets the tilt joint angle to the specified value.

        :param tilt: value to be set for the tilt joint
        :param wait: wait until the tilt angle is set to
                     the target angle.

        :type tilt: float
        :type wait: bool
        """
        tilt = constrain_within_range(
            np.mod(tilt + np.pi, 2 * np.pi) - np.pi,
            self.configs.MIN_TILT,
            self.configs.MAX_TILT,
        )
        self.set_tilt_pub.publish(tilt)
        if wait:
            for i in range(30):
                rospy.sleep(0.1)
                if np.fabs(self.get_tilt() - tilt) < self.tol:
                    break

    def set_pan_tilt(self, pan, tilt, wait=True):
        """
        Sets both the pan and tilt joint angles to the specified values.

        :param pan: value to be set for pan joint
        :param tilt: value to be set for the tilt joint
        :param wait: wait until the pan and tilt angles are set to
                     the target angles.

        :type pan: float
        :type tilt: float
        :type wait: bool
        """
        pan = constrain_within_range(
            np.mod(pan + np.pi, 2 * np.pi) - np.pi,
            self.configs.MIN_PAN,
            self.configs.MAX_PAN,
        )
        tilt = constrain_within_range(
            np.mod(tilt + np.pi, 2 * np.pi) - np.pi,
            self.configs.MIN_TILT,
            self.configs.MAX_TILT,
        )
        self.set_pan_pub.publish(pan)
        self.set_tilt_pub.publish(tilt)
        if wait:
            for i in range(30):
                rospy.sleep(0.1)
                if (
                    np.fabs(self.get_pan() - pan) < self.tol
                    and np.fabs(self.get_tilt() - tilt) < self.tol
                ):
                    break

    def reset(self):
        """
        This function resets the pan and tilt joints by actuating
        them to their home configuration.
        """
        self.set_pan_tilt(self.configs.RESET_PAN, self.configs.RESET_TILT)

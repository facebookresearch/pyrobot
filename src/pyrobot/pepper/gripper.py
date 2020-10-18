#!/usr/bin/env python
# coding: utf-8

import rospy
import threading
from pyrobot.core import Gripper
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class PepperGripper(Gripper):
    """
    This is a gripper class for the Pepper robot
    """

    def __init__(self, configs):
        """
        Constructor for the PepperGripper class.

        :param configs: configurations for gripper
        :type configs: YACS CfgNode
        """
        super(PepperGripper, self).__init__(configs=configs)

        # Angles of Pepper's gripper: [r_gripper, l_gripper]
        self.gripper_angles = [0.0, 0.0]
        self.joint_states_lock = threading.Lock()

        self.gripper_pub = rospy.Publisher(
            self.configs.GRIPPER.ROSTOPIC_SET_JOINT,
            JointAnglesWithSpeed,
            queue_size=10)

    def open(self, r_hand=True, speed=0.6):
        """
        Fully opens Pepper's gripper

        :param r_hand: The right hand will be controlled if True, the left one
                       otherwise
        :param speed: The percentage of the max speed to be used while setting
                      the angle (bound between 0.0 and 1.0)
        :type r_hand: bool
        :type speed: float
        """
        self.set_angle(
            self.configs.GRIPPER.GRIPPER_OPENED_POSITION,
            r_hand=r_hand,
            speed=speed)

    def close(self, r_hand=True, speed=0.6):
        """
        Fully closes Pepper's gripper

        :param r_hand: The right hand will be controlled if True, the left one
                       otherwise
        :param speed: The percentage of the max speed to be used while setting
                      the angle (bound between 0.0 and 1.0)
        :type r_hand: bool
        :type speed: float
        """
        self.set_angle(
            self.configs.GRIPPER.GRIPPER_CLOSED_POSITION,
            r_hand=r_hand,
            speed=speed)

    def set_angle(self, angle, r_hand=True, speed=0.6):
        """
        Apply a specific angle on the gripper

        :param angle: The angle to be applied in radians (will be bound between
                      GRIPPER_CLOSED_POSITION and GRIPPER_OPENED_POSITION)
        :param r_hand: The right hand will be controlled if True, the left one
                       otherwise
        :param speed: The percentage of the max speed to be used while setting
                      the angle (bound between 0.0 and 1.0)
        :type angle: float
        :type r_hand: bool
        :type speed: float
        """
        msg = JointAnglesWithSpeed()

        # Choose between the right or left hand
        if r_hand:
            msg.joint_names = [self.configs.GRIPPER.R_GRIPPER_JOINT_NAME]
        else:
            msg.joint_names = [self.configs.GRIPPER.L_GRIPPER_JOINT_NAME]

        # Clip the angle value and the speed value
        angle = max(angle, self.configs.GRIPPER.GRIPPER_CLOSED_POSITION)
        angle = min(angle, self.configs.GRIPPER.GRIPPER_OPENED_POSITION)
        speed = max(speed, 0.0)
        speed = min(speed, 1.0)

        # Build the content of the message and publish it
        msg.joint_angles = [angle]
        msg.speed = speed
        self.gripper_pub.publish(msg)

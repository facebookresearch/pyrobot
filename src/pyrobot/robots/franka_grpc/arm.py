from __future__ import print_function

import copy
import importlib
import os
import sys
import threading
import time

import numpy as np
import tf

import pyrobot.utils.util as prutil

from pyrobot.robots.arm import Arm
from fair_controller_manager import RobotInterface

import torch

class FrankaGRPCArm(Arm):
    """
    This is a parent class on which the robot
    specific Arm classes would be built.
    """
    def __init__(self, configs):
        self.configs = configs
        self.robot = RobotInterface(
            ip_address=self.configs.IP_ADDRESS,
        )
        # set home pose

    def go_home(self):
        """
        Reset robot to default home position
        """
        self.robot.go_home()

    def get_joint_angles(self):
        """
        Return the joint angles

        :return: joint_angles
        :rtype: np.ndarray
        """
        return self.robot.get_joint_angles().numpy()

    def get_joint_velocities(self):
        """
        Return the joint velocities

        :return: joint_vels
        :rtype: np.ndarray
        """
        return self.robot.get_joint_velocities().numpy()

    def get_joint_torques(self):
        """
        Return the joint torques

        :return: joint_torques
        :rtype: np.ndarray
        """
        raise NotImplementedError

    def get_joint_angle(self, joint):
        """
        Return the joint angle of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint angle
        :rtype: float
        """
        raise NotImplementedError

    def get_joint_velocity(self, joint):
        """
        Return the joint velocity of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint velocity
        :rtype: float
        """
        raise NotImplementedError

    def get_joint_torque(self, joint):
        """
        Return the joint torque of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint torque
        :rtype: float
        """
        raise NotImplementedError

    def set_joint_positions(self, positions, **kwargs):
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
        positions = torch.Tensor(positions)
        self.robot.set_joint_positions(positions)

    def set_joint_velocities(self, velocities, **kwargs):
        """
        Sets the desired joint velocities for all arm joints

        :param velocities: target joint velocities
        :type velocities: list
        """
        raise NotImplementedError

    def set_joint_torques(self, torques, **kwargs):
        """
        Sets the desired joint torques for all arm joints

        :param torques: target joint torques
        :type torques: list
        """
        raise NotImplementedError

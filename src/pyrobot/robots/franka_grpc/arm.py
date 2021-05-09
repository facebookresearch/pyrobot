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
import torchcontrol as toco


class MyPDPolicy(toco.PolicyModule):
    """
    Custom policy that performs PD control around a desired joint position
    """

    def __init__(self, desired_joint_pos, kq, kqd, **kwargs):
        """
        Args:
            desired_joint_pos (int):    Number of steps policy should execute
            hz (double):                Frequency of controller
            kq, kqd (torch.Tensor):     PD gains (1d array)
        """
        super().__init__(**kwargs)

        self.q_desired = torch.nn.Parameter(desired_joint_pos)

        # Initialize modules
        self.feedback = toco.modules.JointSpacePD(kq, kqd)

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        # Parse states
        q_current = state_dict["joint_pos"]
        qd_current = state_dict["joint_vel"]

        # Execute PD control
        output = self.feedback(
            q_current, qd_current, self.q_desired, torch.zeros_like(qd_current)
        )

        return {"torque_desired": output}


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
        self.robot.set_home_pose(torch.Tensor(self.configs.NEUTRAL_POSE))

        q_initial = self.robot.get_joint_angles()
        default_kq = torch.Tensor(self.robot.metadata.default_Kq)
        default_kqd = torch.Tensor(self.robot.metadata.default_Kqd)
        policy = MyPDPolicy(
            desired_joint_pos=q_initial,
            kq=default_kq,
            kqd=default_kqd,
        )
        self.robot.send_torch_policy(policy, blocking=False)

        self.joint_update_initialized = True

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
        positions = torch.Tensor(positions)
        if wait:
            self.robot.set_joint_positions(positions)
            self.joint_update_initialized = False
        else:
            if not self.joint_update_initialized:
                q_initial = self.robot.get_joint_angles()
                default_kq = torch.Tensor(self.robot.metadata.default_Kq)
                default_kqd = torch.Tensor(self.robot.metadata.default_Kqd)
                policy = MyPDPolicy(
                    desired_joint_pos=q_initial,
                    kq=default_kq,
                    kqd=default_kqd,
                )
                self.robot.send_torch_policy(policy, blocking=False)
            self.robot.update_current_policy({"q_desired": positions})


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

import numpy as np
import math
import pyrobot.utils.util as prutil
import rospy
from tf.transformations import euler_from_quaternion, euler_from_matrix

from pyrep.robots.mobiles.nonholonomic_base import NonHolonomicBase
from pyrep.errors import ConfigurationPathError
from pyrep.robots.arms.arm import Arm
import pyrobot.utils.util as prutil
from pyrep.objects.object import Object

from pyrep.errors import ConfigurationError, ConfigurationPathError, IKError
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
import copy
from pyrep.robots.end_effectors.locobot_gripper import LoCoBotGripper as VREP_Gripper


class LoCoBotGripper:
    """
    Interface for gripper
    """

    def __init__(self, configs, simulator, wait_time=3):
        """
        The constructor for LoCoBotGripper class.

        :param configs: configurations for gripper
        :param wait_time: waiting time for opening/closing gripper
        :type configs: YACS CfgNode
        :type wait_time: float
        """
        self.configs = configs
        self.sim = simulator.sim
        self.gripper = VREP_Gripper()
        self.open()

    def get_gripper_state(self):
        """
        Return the gripper state. 

        :return: state

                 state = -1: unknown gripper state

                 state = 0: gripper is fully open

                 state = 1: gripper is closing

                 state = 2: there is an object in the gripper

                 state = 3: gripper is fully closed

        :rtype: int
        """
        if self.gripper.get_open_ammount() is 1:
            return 0
        elif self.gripper.get_open_ammount() is 0:
            return 3
        else:
            return 2

    def open(self, wait=True):
        """
        Commands gripper to open fully

        :param wait: True if blocking call and will return
                     after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.gripper.actuate(1, 1)

    def close(self, wait=True):
        """
        Commands gripper to close fully

        :param wait: True if blocking call and will return
                     after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.gripper.actuate(0, 1)

    def reset(self, wait=True):
        """
        Utility function to reset gripper if it is stuck

        :param wait: True if blocking call and will return
                     after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.open(wait)
        self.close(wait)
        self.open(wait)

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import intera_interface
import rospy
from pyrobot.core import Gripper


class SawyerGripper(Gripper):
    """
    Interface for gripper.
    """

    def __init__(self, configs, ee_name="right_gripper", calibrate=True, wait_time=3):
        """
        :param configs: configurations for the robot
        :param ee_name: robot gripper name (default: "right_gripper")
        :param calibrate: Attempts to calibrate the gripper when
                          initializing class (defaults True)
        :param wait_time: waiting time for opening/closing gripper
        :type configs: YACS CfgNode
        :type ee_name: str
        :type calibrate: bool
        :type wait_time: float
        """
        super(SawyerGripper, self).__init__(configs=configs)
        self.wait_time = wait_time
        self.gripper = intera_interface.Gripper(ee_name=ee_name, calibrate=calibrate)

    def open(self, position=None, wait=True):
        """
        Commands gripper to open fully

        :param position: gripper position
        :type position: float
        :param wait: True if blocking call and will return after
                     target_joint is achieved, False otherwise
        :type wait: bool
        """
        if position is None:
            position = self.configs.GRIPPER.GRIPPER_MAX_POSITION
        self.gripper.gripper_io.set_signal_value("position_m", position)
        if wait:
            rospy.sleep(self.wait_time)

    def reset(self, wait=True):
        """
        Utility function to reset gripper if it is stuck

        :param wait: True if blocking call and will return after
                     target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.open(wait)
        self.close(wait)
        self.open(wait)

    def close(self, position=None, wait=True):
        """
        Commands gripper to close fully

        :param position: gripper position
        :type position: float
        :param wait: True if blocking call and will return after
                     target_joint is achieved, False otherwise
        :type wait: bool
        """
        if position is None:
            position = self.configs.GRIPPER.GRIPPER_MIN_POSITION
        self.gripper.gripper_io.set_signal_value("position_m", position)
        if wait:
            rospy.sleep(self.wait_time)

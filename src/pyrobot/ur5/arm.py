# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import numpy as np
import rospy

from pyrobot.core import Arm


class UR5Arm(Arm):
    """
    This class has functionality to control a UR5 manipulator.
    """

    def __init__(self, configs, moveit_planner="ESTkConfigDefault"):
        """
        The constructor for LoCoBotArm class.

        :param configs: configurations read from config file
        :param moveit_planner: Planner name for moveit,
                               only used if planning_mode = 'moveit'.
        :type configs: YACS CfgNode
        :type moveit_planner: string
        """
        super(UR5Arm, self).__init__(
            configs=configs, moveit_planner=moveit_planner, use_moveit=True
        )

    def go_home(self):
        """
        Commands robot to home position
        """
        self.set_joint_positions(np.zeros(self.arm_dof), plan=True)

    def move_to_neutral(self):
        """
        Move the robot to a pre-defined neutral pose
        """

        # TODO: Change it to some better neutral position
        neutral_pos = [-0.002, -0.9005, 1.9278, -1.0271, -0.0009, 0.0031]
        self.set_joint_positions(neutral_pos, plan=True)

    def _setup_joint_pub(self):
        rospy.loginfo("Setting up the joint publishers")

    def _pub_joint_positions(self, positions):
        raise NotImplementedError

    def _pub_joint_velocities(self, velocities):
        raise NotImplementedError("Velocity control for " "UR not supported yet!")

    def _pub_joint_torques(self, torques):
        raise NotImplementedError("Torque control for " "UR is not supported yet!")

    def set_joint_velocities(self, velocities, **kwargs):
        raise NotImplementedError("Velocity control for " "UR not supported yet!")

    def set_joint_positions(self, positions, plan=True, wait=True, **kwargs):
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
        result = False
        if isinstance(positions, np.ndarray):
            positions = positions.flatten().tolist()
        if plan:
            if not self.use_moveit:
                raise ValueError(
                    "Moveit is not initialized, " "did you pass in use_moveit=True?"
                )
            if isinstance(positions, np.ndarray):
                positions = positions.tolist()

            rospy.loginfo("Moveit Motion Planning...")
            result = self.moveit_group.moveToJointPosition(
                self.arm_joint_names, positions, wait=wait
            )
        else:
            raise NotImplementedError

            # self._pub_joint_positions(positions)
            # if wait:
            #     result = self._loop_angle_pub_cmd(self._pub_joint_positions,
            #                                       positions)
        return result

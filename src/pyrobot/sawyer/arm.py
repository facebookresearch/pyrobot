# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import intera_interface
import numpy as np
import rospy
from intera_core_msgs.msg import (
    JointCommand,
    CollisionDetectionState,
)
from intera_interface import CHECK_VERSION
from pyrobot.core import Arm


class SawyerArm(Arm):
    """
    This class has functionality to control a Sawyer manipulator.
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
        super(SawyerArm, self).__init__(
            configs=configs, moveit_planner=moveit_planner, use_moveit=True
        )
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        rs.enable()
        cos_state = self.configs.ARM.ROSTOPIC_COLLISION_STATE
        self.collision_state_sub = rospy.Subscriber(
            cos_state, CollisionDetectionState, self._callback_collision, queue_size=1
        )

    def go_home(self):
        """
        Commands robot to home position
        """
        self.set_joint_positions(np.zeros(self.arm_dof), plan=False)

    def move_to_neutral(self):
        """
        Move the robot to a pre-defined neutral pose
        """
        neutral_pos = [0.002, -1.182, 0.002, 2.177, 0.001, 0.567, 3.316]
        self.set_joint_positions(neutral_pos, plan=False)

    def get_collision_state(self):
        """
        Return the collision state

        :return: collision or not

        :rtype: bool
        """
        return self._collision_state

    def _setup_joint_pub(self):
        self.joint_pub = rospy.Publisher(
            self.configs.ARM.ROSTOPIC_SET_JOINT, JointCommand, queue_size=1
        )

    def _callback_collision(self, data):
        self._collision_state = data.collision_state

    def _pub_joint_positions(self, positions):
        command_msg = JointCommand()
        command_msg.names = self.arm_joint_names
        command_msg.position = positions
        command_msg.mode = JointCommand.POSITION_MODE
        command_msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(command_msg)

    def _pub_joint_velocities(self, velocities):
        command_msg = JointCommand()
        command_msg.names = self.arm_joint_names
        command_msg.velocity = velocities
        command_msg.mode = JointCommand.VELOCITY_MODE
        command_msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(command_msg)

    def _pub_joint_torques(self, torques):
        command_msg = JointCommand()
        command_msg.names = self.arm_joint_names
        command_msg.effort = torques
        command_msg.mode = JointCommand.TORQUE_MODE
        command_msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(command_msg)

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import rospy
from pyrobot.core import Arm


class UrArm(Arm):
    """
    This class has functionality to control a Sawyer manipulator.
    """

    def __init__(self,
                 configs,
                 moveit_planner='ESTkConfigDefault'):
        """
        The constructor for LoCoBotArm class.

        :param configs: configurations read from config file
        :param moveit_planner: Planner name for moveit,
                               only used if planning_mode = 'moveit'.
        :type configs: YACS CfgNode
        :type moveit_planner: string
        """
        super(UrArm, self).__init__(configs=configs,
                                        moveit_planner=moveit_planner,
                                        use_moveit=True)
                                        
    def go_home(self):
        """
        Commands robot to home position
        """
        self.set_joint_positions(np.zeros(self.arm_dof), plan=True)

    def move_to_neutral(self):
        """
        Move the robot to a pre-defined neutral pose
        """
        # TODO: Need to find good neutral position
        neutral_pos = np.zeros(self.arm_dof)
        neutral_pos[1], neutral_pos[2] = -np.pi/2, np.pi/2

        self.set_joint_positions(neutral_pos, plan=True)

    def get_collision_state(self):
        """
        Return the collision state

        :return: collision or not

        :rtype: bool
        """
        return self._collision_state

    def _setup_joint_pub(self):
        raise NotImplementedError
    
    def _pub_joint_positions(self, positions):
        """
        TODO: Figure out way to set joint angles 
        """
        raise NotImplementedError

    def _pub_joint_velocities(self, velocities):
        """
        TODO: Figure out way to set velocity
        """
        raise NotImplementedError

    def _pub_joint_torques(self, torques):
        """
        UR doesnt allow to set torque on joints
        """
        raise NotImplementedError
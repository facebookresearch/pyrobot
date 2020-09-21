# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Copyright 2011-2014, Michael Ferguson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import actionlib
from tf.listener import TransformListener
from geometry_msgs.msg import *
from moveit_pybind import Plan, MoveGroupInterfaceWrapper, Pose, RobotTrajectory
import numpy as np

class MoveGroupInterface(object):
    """
    This class lets you interface with the movegroup node. It provides
    the ability to execute the specified trajectory on the robot by
    communicating to the movegroup node using services.
    """

    def __init__(
        self,
        group,
        robot_param,
        fixed_frame,
        gripper_frame,
        listener=None,
        plan_only=False,
    ):

        self.move_group_interface = MoveGroupInterfaceWrapper(
            group,
            robot_param,
        )
        self.plan_only = plan_only

        self.planner_id = None
        self.planning_time = 15.0
        self.ee_frame = gripper_frame

    def moveToJointPosition(
        self, joints, positions, tolerance=0.01, wait=True, **kwargs
    ):
        """
        Move the arm to set of joint position goals

        :param joints: joint names for which the target position
                is specified.
        :param positions: target joint positions
        :param tolerance: allowed tolerance in the final joint positions.
        :param wait: if enabled, makes the fuctions wait until the
            target joint position is reached

        :type joints: list of string element type
        :type positions: list of float element type
        :type tolerance: float
        :type wait: bool
        """

        # Check arguments
        p_out = self.motionPlanToJointPosition(joints, positions, tolerance, wait)
 
        self.move_group_interface.execute(p_out)
        return p_out

    def moveToPose(self, position, orientation, tolerance=0.01, wait=True, **kwargs):

        """
        Move the arm, based on a goal pose_stamped for the end effector.

        :param pose: target pose to which we want to move
                            specified link to
        :param tolerance: allowed tolerance in the final joint positions.
        :param wait: if enabled, makes the fuctions wait until the
            target joint position is reached

        :type pose_stamped: ros message object of type PoseStamped
        :type gripper_frame: string
        :type tolerance: float
        :type wait: bool
        """

        # Check arguments
        self.move_group_interface.clearPoseTargets()
        position = np.array(position).flatten()
        orientation = np.array(orientation).flatten()

        if orientation.size == 4:
            orientation = orientation.flatten()
            ori_x = orientation[0]
            ori_y = orientation[1]
            ori_z = orientation[2]
            ori_w = orientation[3]
        elif orientation.size == 3:
            quat = prutil.euler_to_quat(orientation)
            ori_x = quat[0]
            ori_y = quat[1]
            ori_z = quat[2]
            ori_w = quat[3]
        elif orientation.size == 9:
            orientation = orientation.reshape((3, 3))
            quat = prutil.rot_mat_to_quat(orientation)
            ori_x = quat[0]
            ori_y = quat[1]
            ori_z = quat[2]
            ori_w = quat[3]
        else:
            raise TypeError(
                "Orientation must be in one "
                "of the following forms:"
                "rotation matrix, euler angles, or quaternion"
            )

        p_in = Pose()
        p_in.position.x = position[0]
        p_in.position.y = position[1]
        p_in.position.z = position[2]
        p_in.orientation.x = ori_x
        p_in.orientation.y = ori_y
        p_in.orientation.z = ori_z
        p_in.orientation.w = ori_w

        p_out = self.motionPlanToPose(p_in)
        self.move_group_interface.execute(p_out)

        return p_out

    def motionPlanToJointPosition(
        self, joints, positions, tolerance=0.01, wait=True, **kwargs
    ):
        """
        Move the arm to set of joint position goals

        :param joints: joint names for which the target position
                is specified.
        :param positions: target joint positions
        :param tolerance: allowed tolerance in the final joint positions.
        :param wait: if enabled, makes the fuctions wait until the
            target joint position is reached

        :type joints: list of string element type
        :type positions: list of float element type
        :type tolerance: float
        :type wait: bool
        """

        self.move_group_interface.clearPoseTargets()
        target_joint = np.array(positions).flatten().tolist()
        success = self.move_group_interface.setJointValueTarget(positions)

        p_out = self.move_group_interface.plan()
        return p_out

    def motionPlanToPose(self, pose):
        """
        Move the arm to set of joint position goals

        :param joints: joint names for which the target position
                is specified.
        :param positions: target joint positions
        :param tolerance: allowed tolerance in the final joint positions.
        :param wait: if enabled, makes the fuctions wait until the
            target joint position is reached

        :type joints: list of string element type
        :type positions: list of float element type
        :type tolerance: float
        :type wait: bool
        """

        p_in = Pose()
        p_in.position.x = pose.position.x
        p_in.position.y = pose.position.y
        p_in.position.z = pose.position.z
        p_in.orientation.x = pose.orientation.x
        p_in.orientation.y = pose.orientation.y
        p_in.orientation.z = pose.orientation.z
        p_in.orientation.w = pose.orientation.w

        success = self.move_group_interface.setPoseTarget(p_in, self.ee_frame)
        p_out = self.move_group_interface.plan()
        return p_out

    def setPlannerId(self, planner_id):
        """
        Sets the planner_id used for all future planning requests.
        :param planner_id: The string for the planner id, set to None to clear
        """
        self.move_group_interface.setPlannerId(planner_id)

    def setPlanningTime(self, time):
        """
        Set default planning time to be used for future planning request.
        """
        self.move_group_interface.setPlanningTime(time)
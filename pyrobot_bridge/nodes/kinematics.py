#!/usr/bin/env python

#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import PyKDL as kdl
import numpy as np
import copy
import rospy

import tf
from geometry_msgs.msg import Twist
from kdl_parser_py.urdf import treeFromParam
from sensor_msgs.msg import JointState
from trac_ik_python import trac_ik

from pyrobot_bridge.srv import *


class Kinematics(object):
    """docstring for Kinematics"""

    def __init__(self):
        self.init_done = False
        rospy.init_node("pyrobot_kinematics")

        self.ik_srv = rospy.Service("pyrobot/ik", IkCommand, self.ik_server)
        self.fk_srv = rospy.Service("pyrobot/fk", FkCommand, self.fk_server)
        rospy.spin()

    def _init_kinematics(self):
        # Ros-Params
        base_link = rospy.get_param("pyrobot/base_link")
        gripper_link = rospy.get_param("pyrobot/gripper_link")
        robot_description = rospy.get_param("pyrobot/robot_description")

        urdf_string = rospy.get_param(robot_description)
        self.num_ik_solver = trac_ik.IK(
            base_link, gripper_link, urdf_string=urdf_string
        )

        _, self.urdf_tree = treeFromParam(robot_description)
        self.urdf_chain = self.urdf_tree.getChain(base_link, gripper_link)
        self.arm_joint_names = self._get_kdl_joint_names()
        self.arm_link_names = self._get_kdl_link_names()
        self.arm_dof = len(self.arm_joint_names)

        self.jac_solver = kdl.ChainJntToJacSolver(self.urdf_chain)
        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)
        self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)

    def _get_kdl_link_names(self):
        num_links = self.urdf_chain.getNrOfSegments()
        link_names = []
        for i in range(num_links):
            link_names.append(self.urdf_chain.getSegment(i).getName())
        return copy.deepcopy(link_names)

    def _get_kdl_joint_names(self):
        num_links = self.urdf_chain.getNrOfSegments()
        num_joints = self.urdf_chain.getNrOfJoints()
        joint_names = []
        for i in range(num_links):
            link = self.urdf_chain.getSegment(i)
            joint = link.getJoint()
            joint_type = joint.getType()
            # JointType definition: [RotAxis,RotX,RotY,RotZ,TransAxis,
            #                        TransX,TransY,TransZ,None]
            if joint_type > 1:
                continue
            joint_names.append(joint.getName())
        assert num_joints == len(joint_names)
        return copy.deepcopy(joint_names)

    def ik_server(self, req):

        if not self.init_done:
            self._init_kinematics()
            self.init_done = True

        resp = IkCommandResponse()

        if (
            len(req.pose) < 7
            or len(req.tolerance) < 6
            or len(req.init_joint_positions) != self.arm_dof
        ):
            resp.success = False
            rospy.logerr("Incorrect IK service request. Please fix it.")
            return resp

        joint_positions = self.num_ik_solver.get_ik(
            req.init_joint_positions,
            req.pose[0],
            req.pose[1],
            req.pose[2],
            req.pose[3],
            req.pose[4],
            req.pose[5],
            req.pose[6],
            req.tolerance[0],
            req.tolerance[1],
            req.tolerance[2],
            req.tolerance[3],
            req.tolerance[4],
            req.tolerance[5],
        )

        if joint_positions is None:
            resp.success = False
            rospy.logwarn("Failed to find find an IK solution.")
            return resp

        resp.joint_positions = joint_positions
        resp.success = True

        return resp

    def _kdl_frame_to_numpy(self, frame):
        p = frame.p
        M = frame.M
        return np.array(
            [
                [M[0, 0], M[0, 1], M[0, 2], p.x()],
                [M[1, 0], M[1, 1], M[1, 2], p.y()],
                [M[2, 0], M[2, 1], M[2, 2], p.z()],
                [0, 0, 0, 1],
            ]
        )

    def rot_mat_to_quat(self, rot):
        """
		Convert the rotation matrix into quaternion.
		:param quat: the rotation matrix (shape: :math:`[3, 3]`)
		:type quat: numpy.ndarray
		:return: quaternion [x, y, z, w] (shape: :math:`[4,]`)
		:rtype: numpy.ndarray
		"""
        R = np.eye(4)
        R[:3, :3] = rot
        return tf.transformations.quaternion_from_matrix(R)

    def joints_to_kdl(self, joint_values):
        """
		Convert the numpy array into KDL data format
		:param joint_values: values for the joints
		:return: kdl data type
		"""
        num_jts = joint_values.size
        kdl_array = kdl.JntArray(num_jts)
        for idx in range(num_jts):
            kdl_array[idx] = joint_values[idx]
        return kdl_array

    def fk_server(self, req):
        """
		Given joint angles, compute the pose of desired_frame with respect
		to the base frame (self.configs.ARM.ARM_BASE_FRAME). The desired frame
		must be in self.arm_link_names
		:param joint_positions: joint angles
		:param des_frame: desired frame
		:type joint_positions: np.ndarray
		:type des_frame: string
		:return: translational vector and rotational matrix
		:rtype: np.ndarray, np.ndarray
		"""
        if not self.init_done:
            self._init_kinematics()
            self.init_done = True

        resp = FkCommandResponse()

        joint_positions = np.asarray(req.joint_angles).flatten()
        des_frame = req.end_frame
        if joint_positions.size != self.arm_dof:
            rospy.logerr("Forward Kinenatics: Invalid length of joint angles!")
            resp.success = False
            return resp

        kdl_jnt_angles = self.joints_to_kdl(joint_positions)

        kdl_end_frame = kdl.Frame()
        idx = self.arm_link_names.index(des_frame) + 1
        fg = self.fk_solver_pos.JntToCart(kdl_jnt_angles, kdl_end_frame, idx)

        if fg < 0:
            rospy.logerr("Forward Kinenatics: KDL Pos JntToCart error!")
            resp.success = False
            return resp

        pose = self._kdl_frame_to_numpy(kdl_end_frame)
        pos = pose[:3, 3].reshape(-1, 1)
        resp.pos = list(pos)
        resp.quat = list(self.rot_mat_to_quat(pose[:3, :3]))
        resp.success = True
        return resp


if __name__ == "__main__":
    server = Kinematics()

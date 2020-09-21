#!/usr/bin/env python

#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import kdl_pybind as kdl
import urdf_parser_py.urdf as urdf
import numpy as np
import copy
import rospy

import tf
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from tf_conversions import posemath

class Kinematics(object):
    """docstring for Kinematics"""

    def __init__(self, base_link, gripper_link, robot_description):
        # Ros-Params
        urdf_string = rospy.get_param(robot_description)

        _, self.urdf_tree = treeFromParam(robot_description)
        self.urdf_chain = self.urdf_tree.getChain(base_link, gripper_link)
        self.arm_joint_names = self._get_kdl_joint_names()
        self.arm_link_names = self._get_kdl_link_names()
        self.arm_dof = len(self.arm_joint_names)

        self.jac_solver = kdl.ChainJntToJacSolver(self.urdf_chain)
        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)
        self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.urdf_chain)

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
            if int(joint_type) > 1:
                continue
            joint_names.append(joint.getName())
        assert num_joints == len(joint_names)
        return copy.deepcopy(joint_names)

    def ik(self, pose, tolerance, init_joint_positions):

        assert (
            len(pose) >= 7
            and len(tolerance) >= 6
            and len(init_joint_positions) == self.arm_dof
        ), "Incorrect IK request. Please fix it."

        q_init = self.joints_to_kdl(init_joint_positions)
        p_in = Pose()
        p_in.position.x = pose[0]
        p_in.position.y = pose[1]
        p_in.position.z = pose[2]
        p_in.orientation.x = pose[3]
        p_in.orientation.y = pose[4]
        p_in.orientation.z = pose[5]
        p_in.orientation.w = pose[6]
        p_in = posemath.fromMsg(p_in)
        q_out = kdl.JntArray(len(list(q_init)))
        ig = self.ik_solver.CartToJnt(q_init, p_in, q_out)

        assert ig >= 0, "Inverse Kinenatics: KDL Pos JntToCart error!"

        joint_positions = list(q_out)

        return joint_positions

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
        num_jts = len(joint_values)
        kdl_array = kdl.JntArray(num_jts)
        for idx in range(num_jts):
            kdl_array[idx] = joint_values[idx]
        return kdl_array

    def fk(self, joint_positions, des_frame):
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

        joint_positions = np.asarray(joint_positions).flatten()
        assert joint_positions.size == self.arm_dof, "Forward Kinenatics: Invalid length of joint angles!"

        kdl_jnt_angles = self.joints_to_kdl(joint_positions)

        kdl_end_frame = kdl.Frame()
        idx = self.arm_link_names.index(des_frame) + 1
        fg = self.fk_solver_pos.JntToCart(kdl_jnt_angles, kdl_end_frame, idx)

        assert fg >= 0, "Forward Kinenatics: KDL Pos JntToCart error!"

        pose = self._kdl_frame_to_numpy(kdl_end_frame)
        pos = pose[:3, 3].reshape(-1, 1)
        pos = list(pos)
        quat = list(self.rot_mat_to_quat(pose[:3, :3]))
        return pos, quat

###############
# Helper Func #
###############

def treeFromFile(filename):
    """
    Construct a PyKDL.Tree from an URDF file.
    :param filename: URDF file path
    """

    with open(filename) as urdf_file:
        return treeFromUrdfModel(urdf.URDF.from_xml_string(urdf_file.read()))

def treeFromParam(param):
    """
    Construct a PyKDL.Tree from an URDF in a ROS parameter.
    :param param: Parameter name, ``str``
    """

    return treeFromUrdfModel(urdf.URDF.from_parameter_server(param))

def treeFromString(xml):
    """
    Construct a PyKDL.Tree from an URDF xml string.
    :param xml: URDF xml string, ``str``
    """

    return treeFromUrdfModel(urdf.URDF.from_xml_string(xml))

def _toKdlPose(pose):
    # URDF might have RPY OR XYZ unspecified. Both default to zeros
    rpy = pose.rpy if pose and pose.rpy and len(pose.rpy) == 3 else [0, 0, 0]
    xyz = pose.xyz if pose and pose.xyz and len(pose.xyz) == 3 else [0, 0, 0]

    return kdl.Frame(
          kdl.Rotation.RPY(*rpy),
          kdl.Vector(*xyz))


def _toKdlInertia(i):
    # kdl specifies the inertia in the reference frame of the link, the urdf
    # specifies the inertia in the inertia reference frame
    origin = _toKdlPose(i.origin)
    inertia = i.inertia
    return origin.M * kdl.RigidBodyInertia(
            i.mass, origin.p,
            kdl.RotationalInertia(inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz));

def _toKdlJoint(jnt):

    fixed = lambda j,F: kdl.Joint(j.name, getattr(kdl.Joint, 'None'))
    rotational = lambda j,F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.RotAxis)
    translational = lambda j,F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.TransAxis)

    type_map = {
            'fixed': fixed,
            'revolute': rotational,
            'continuous': rotational,
            'prismatic': translational,
            'floating': fixed,
            'planar': fixed,
            'unknown': fixed,
            }

    return type_map[jnt.type](jnt, _toKdlPose(jnt.origin))

def _add_children_to_tree(robot_model, root, tree):


    # constructs the optional inertia
    inert = kdl.RigidBodyInertia(0)
    if root.inertial:
        inert = _toKdlInertia(root.inertial)

    # constructs the kdl joint
    (parent_joint_name, parent_link_name) = robot_model.parent_map[root.name]
    parent_joint = robot_model.joint_map[parent_joint_name]

    # construct the kdl segment
    sgm = kdl.Segment(
        root.name,
        _toKdlJoint(parent_joint),
        _toKdlPose(parent_joint.origin),
        inert)

    # add segment to tree
    if not tree.addSegment(sgm, parent_link_name):
        return False

    if root.name not in robot_model.child_map:
        return True

    children = [robot_model.link_map[l] for (j,l) in robot_model.child_map[root.name]]

    # recurslively add all children
    for child in children:
        if not _add_children_to_tree(robot_model, child, tree):
            return False

    return True;

def treeFromUrdfModel(robot_model, quiet=False):
    """
    Construct a PyKDL.Tree from an URDF model from urdf_parser_python.
    :param robot_model: URDF xml string, ``str``
    :param quiet: If true suppress messages to stdout, ``bool``
    """

    root = robot_model.link_map[robot_model.get_root()]

    if root.inertial and not quiet:
        print("The root link %s has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF." % root.name);

    ok = True
    tree = kdl.Tree(root.name)

    #  add all children
    for (joint,child) in robot_model.child_map[root.name]:
        if not _add_children_to_tree(robot_model, robot_model.link_map[child], tree):
            ok = False
            break

    return (ok, tree)

if __name__ == "__main__":
    server = Kinematics()

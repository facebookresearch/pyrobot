#!/usr/bin/env python

from __future__ import print_function



import PyKDL as kdl
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from kdl_parser_py.urdf import treeFromParam
from sensor_msgs.msg import JointState
from trac_ik_python import trac_ik

from pyrobot_bridge.srv import *


class Kinematics(object):
	"""docstring for Kinematics"""
	def __init__(self):
		self.init_done = False
		
		rospy.init_node('pyrobot_kinematics')
		self.ik_srv = rospy.Service('pyrobot_ik', IkCommand, self.ik_server)


		rospy.spin()

	def _init_kinematics(self):




		robot_description = '/robot_description' #todo: change this to rosparam
		urdf_string = rospy.get_param(robot_description)
		self.num_ik_solver = trac_ik.IK('base_link', #todo: change this to rosparam
										'gripper_link', #todo: change this to rosparam
										urdf_string=urdf_string)

		_, self.urdf_tree = treeFromParam(robot_description)
		self.urdf_chain = self.urdf_tree.getChain('base_link',  #todo: change this to rosparam
		                                          'gripper_link')  #todo: change this to rosparam
		#self.arm_joint_names = self._get_kdl_joint_names()
		#self.arm_link_names = self._get_kdl_link_names()
		self.arm_dof = 5#len(self.arm_joint_names)

		self.jac_solver = kdl.ChainJntToJacSolver(self.urdf_chain)
		self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)
		self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)



	def ik_server(self, req):

		if not self.init_done:
			self._init_kinematics()
			self.init_done = True

		resp = IkCommandResponse()

		if len(req.pose) < 7 or len(req.tolerance) < 6 \
			or len(req.init_joint_positions) != self.arm_dof :
			resp.success = False
			rospy.logerr("Incorrect IK service request. Please fix it.")
			return resp 

		joint_positions = self.num_ik_solver.get_ik(req.init_joint_positions,
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
													req.tolerance[5],)

		if joint_positions is None:
			resp.success = False
			rosp.logwarn("Failed to find find an IK solution.")
			return resp 

		resp.joint_positions = joint_positions
		resp.success = True

		return resp



   #  def fk_server(self, req):
   #      """
   #      Given joint angles, compute the pose of desired_frame with respect
   #      to the base frame (self.configs.ARM.ARM_BASE_FRAME). The desired frame
   #      must be in self.arm_link_names
   #      :param joint_positions: joint angles
   #      :param des_frame: desired frame
   #      :type joint_positions: np.ndarray
   #      :type des_frame: string
   #      :return: translational vector and rotational matrix
   #      :rtype: np.ndarray, np.ndarray
   #      """

   #      if !self.init_done:
			# self._init_kinematics()

   #      joint_positions = joint_positions.flatten()
   #      assert joint_positions.size == self.arm_dof
   #      kdl_jnt_angles = prutil.joints_to_kdl(joint_positions)

   #      kdl_end_frame = kdl.Frame()
   #      idx = self.arm_link_names.index(des_frame) + 1
   #      fg = self.fk_solver_pos.JntToCart(kdl_jnt_angles,
   #                                        kdl_end_frame,
   #                                        idx)
   #      assert fg == 0, 'KDL Pos JntToCart error!'
   #      pose = prutil.kdl_frame_to_numpy(kdl_end_frame)
   #      pos = pose[:3, 3].reshape(-1, 1)
   #      rot = pose[:3, :3]
   #      return pos, rot


if __name__ == "__main__":
    server = Kinematics()
from pyrobot.algorithms.motion_planning import MotionPlanner 
from pyrobot.algorithms.kinematics import Kinematics

from moveit_pybind import Plan, MoveGroupInterfaceWrapper, Pose

import pyrobot.utils.util as prutil
import numpy as np

class MoveitKinPlanner(MotionPlanner):
	"""Implementation of moveit!-based motion planning algorithm with kinematics dependency. 
	"""
	def __init__(
		self,
		configs,
		world, 
		ros_launch_manager = None,
		robots={},
		sensors={},
		algorithms={}
	):

		super(MotionPlanner, self).__init__(
										configs, 
										world,
										ros_launch_manager, 
										robots,
										sensors,
										algorithms
										)

		self.robot_label = list(self.robots.keys())[0]
		
		self.move_group_interface = MoveGroupInterfaceWrapper(
			self.robots[self.robot_label]["arm"].configs["MOVEGROUP_NAME"],
			self.robots[self.robot_label]["arm"].configs["ARM_ROBOT_DSP_PARAM_NAME"],
		)
		self.ee_frame = self.robots[self.robot_label]["arm"].configs["EE_FRAME"]

	def plan_end_effector_pose(self, position, orientation):
		target_joint = self.algorithms["Kinematics"].inverse_kinematics(position, orientation)
		
		p_out = self.plan_joint_angles(target_joint)
		return p_out

	def plan_joint_angles(self, target_joint):
		self.move_group_interface.clearPoseTargets()
		target_joint = np.array(target_joint).flatten().tolist()
		success = self.move_group_interface.setJointValueTarget(target_joint)

		p_out = self.move_group_interface.plan()
		print(success, len(p_out.trajectory.joint_trajectory.points), p_out.trajectory.joint_trajectory.points[0].positions)
		self.move_group_interface.execute(p_out)
		return p_out

	def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
		raise NotImplementedError()

	def check_cfg(self):
		assert len(self.robots.keys()) == 1, "One motion planner only handle one arm!"
		robot_label = list(self.robots.keys())[0]

		assert len(self.algorithms.keys()) == 1, "Motion planner only have one dependency!"
		assert list(self.algorithms.keys())[0] == "Kinematics", "Motion planner only depend on Kinematics!"
		assert isinstance(self.algorithms["Kinematics"], Kinematics), "Kinematics module needs to extend Kinematics base class!"

		assert "arm" in self.robots[robot_label].keys(), "Arm required for MotionPlanners!"
		assert "ARM_BASE_FRAME" in self.robots[robot_label]["arm"].configs.keys(), "ARM_BASE_FRAME required for KDL solver!"
		assert "EE_FRAME" in self.robots[robot_label]["arm"].configs.keys(), "EE_FRAME required for KDL solver!"
		assert "ARM_ROBOT_DSP_PARAM_NAME" in self.robots[robot_label]["arm"].configs.keys(), "ARM_ROBOT_DSP_PARAM_NAME required for KDL solver!"

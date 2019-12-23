import numpy as np
import math
import pyrobot.utils.util as prutil
import rospy
import habitat_sim.agent as habAgent
import habitat_sim.utils as habUtils
from habitat_sim.agent.controls import ActuationSpec
import habitat_sim.errors

import quaternion
from tf.transformations import euler_from_quaternion, euler_from_matrix

class LoCoBotBase(object):
	"""docstring for SimpleBase"""
	def __init__(self, configs, simulator):
		self.configs = configs
		self.sim = simulator.sim
		self.agent = \
			self.sim.get_agent(self.configs.COMMON.SIMULATOR.DEFAULT_AGENT_ID)

		self.transform = None
		self.init_state = self.get_full_state()

	def execute_action(self, action):
		# actions = "turn_right" or "turn_left" or "move_forward"
		# returns a bool showing if collided or not
		return self.agent.act(action)

	def get_full_state(self):
		# Returns habitat_sim.agent.AgentState
		# temp_state = self.agent.get_state()
		# temp_state.position = habUtils.quat_rotate_vector(self._fix_transform().inverse(), temp_state.position)
		# temp_state.rotation = self._fix_transform() * temp_state.rotation
		return self.agent.get_state()

	def _fix_transform(self):
		'''Return the fixed transform needed to correct habitat-sim agent co-ordinates'''
		if self.transform is None:
			self.transform = habUtils.quat_from_angle_axis( np.pi/2, 
															np.asarray([0.0, 1.0, 0.0]))
			self.transform = self.transform * habUtils.quat_from_angle_axis( -np.pi/2, 
															np.asarray([1.0,0.0,0.0]))
		return self.transform


	def get_state(self):
		# Returns (x, y, yaw)
		cur_state = self.get_full_state()
		true_position = cur_state.position - self.init_state.position
		true_position = habUtils.quat_rotate_vector(self.init_state.rotation.inverse(), true_position)
		true_position = habUtils.quat_rotate_vector(self._fix_transform().inverse(), true_position)

		true_rotation =   self.init_state.rotation.inverse() * cur_state.rotation
		quat_list = [true_rotation.x, true_rotation.y, true_rotation.z, true_rotation.w]
		(r, yaw, p)  = euler_from_quaternion(quat_list) 

		return (true_position[0], true_position[1], yaw)

	def stop(self):
		raise NotImplementedError("Veclocity control is not supported in Habitat-Sim!!")

	def set_vel(self, fwd_speed, turn_speed, exe_time=1):
		raise NotImplementedError("Veclocity control is not supported in Habitat-Sim!!")


	def go_to_relative(self, xyt_position, use_map=False, close_loop=False, smooth=False):
		"""
		Moves the robot to the robot to given
		goal state relative to its initial pose.

		:param xyt_position: The  relative goal state of the form (x,y,t)
		:param use_map: When set to "True", ensures that controler is
		                using only free space on the map to move the robot.
		:param close_loop: When set to "True", ensures that controler is
		                   operating in open loop by
		                   taking account of odometry.
		:param smooth: When set to "True", ensures that the motion
		               leading to the goal is a smooth one.

		:type xyt_position: list
		:type use_map: bool
		:type close_loop: bool
		:type smooth: bool

		:return: True if successful; False otherwise (timeout, etc.)
		:rtype: bool
		"""

		try:
			if use_map:
				raise NotImplementedError("Using map feature is not yet supported for Habitat-Sim")
			if close_loop:
				raise NotImplementedError("Closed-loop postion control is not supported in Habitat-Sim!")
			if smooth:
				raise NotImplementedError("Smooth position control feature is not yet for Habitat-Sim")
		except Exception as error:
			print(error)
			return False

		(cur_x, cur_y, cur_yaw) = self.get_state()
		abs_yaw = cur_yaw + xyt_position[2]
		return self._go_to_relative_pose(xyt_position[0], xyt_position[1], abs_yaw)


	def go_to_absolute(self, xyt_position, use_map=False, close_loop=False, smooth=False):
		"""
		Moves the robot to the robot to given goal state in the world frame.

		:param xyt_position: The goal state of the form (x,y,t)
		                     in the world (map) frame.
		:param use_map: When set to "True", ensures that controler is using
		                only free space on the map to move the robot.
		:param close_loop: When set to "True", ensures that controler is
		                   operating in open loop by
		                   taking account of odometry.
		:param smooth: When set to "True", ensures that the motion
		               leading to the goal is a smooth one.

		:type xyt_position: list
		:type use_map: bool
		:type close_loop: bool
		:type smooth: bool

		:return: True if successful; False otherwise (timeout, etc.)
		:rtype: bool
		"""

		try:
			if use_map:
				raise NotImplementedError("Using map feature is not yet supported for Habitat-Sim")
			if close_loop:
				raise NotImplementedError("Closed-loop postion control is not supported in Habitat-Sim!")
			if smooth:
				raise NotImplementedError("Smooth position control feature is not yet for Habitat-Sim")
		except Exception as error:
			print(error)
			return False

		(cur_x, cur_y, cur_yaw) = self.get_state()
		rel_x = xyt_position[0] - cur_x
		rel_y = xyt_position[1] - cur_y
		abs_yaw = xyt_position[2]
		return self._go_to_relative_pose(rel_x, rel_y, abs_yaw)

	
	def _act(self, action_name, actuation):
		"""Take the action specified by action_id

		:param action_id: ID of the action. Retreives the action from
		    `agent_config.action_space <AgentConfiguration.action_space>`
		:return: Whether or not the action taken resulted in a collision
		"""
		did_collide = False
		act_spec = ActuationSpec(actuation)
		did_collide = self.agent.controls.action(
			self.agent.scene_node, action_name, act_spec, apply_filter=True)

		return did_collide

	def _go_to_relative_pose(self, rel_x, rel_y, abs_yaw):


		if math.sqrt(rel_x**2 + rel_y**2) > 0.0:
			# rotate to point to (x, y) point
			action_name = "turn_left"
			if rel_y < 0.0:
				action_name = "turn_right"

			v1 = np.asarray([1, 0])
			v2 = np.asarray([rel_x, rel_y])
			cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
			angle = np.arccos(cosine_angle)

			did_collide = self._act(action_name, math.degrees(angle))
			if did_collide:
				print("Error: Collision accured while 1st rotating!")
				return False

			# move to (x,y) point
			did_collide = self._act("move_forward", math.sqrt(rel_x**2 + rel_y**2))
			if did_collide:
				print("Error: Collision accured while moving straight!")
				return False

		# rotate to match the final yaw!
		(cur_x, cur_y, cur_yaw) = self.get_state()
		rel_yaw = abs_yaw - cur_yaw

		action_name = "turn_left"
		if rel_yaw < 0.0:
			action_name = "turn_right"
			rel_yaw *= -1

		did_collide = self._act(action_name, math.degrees(rel_yaw))
		if did_collide:
			print("Error: Collision accured while rotating!")
			return False


		return True


	def track_trajectory(self, states, controls, close_loop):
		"""
		State trajectory that the robot should track.

		:param states: sequence of (x,y,t) states that the robot should track.
		:param controls: optionally specify control sequence as well.
		:param close_loop: whether to close loop on the
		                   computed control sequence or not.

		:type states: list
		:type controls: list
		:type close_loop: bool

		:return: True if successful; False otherwise (timeout, etc.)
		:rtype: bool
		"""
		raise NotImplementedError

import numpy as np
import pyrobot.utils.util as prutil
from pyrobot.core import Base


class SimpleBase(object):
	"""docstring for SimpleBase"""
	def __init__(self, configs, simulator):
		super(SimpleBase, self).__init__(configs, simulator)
		self.sim = simulator

	def execute_action(self, action):
		# actions = "turn_right" or "turn_left" or "move_forward"
		# returns a bool showing if collided or not
		return self.sim._default_agent.act(action)

	def get_state(self):
		# Returns habitat_sim.agent.AgentState
		return self.sim._default_agent.get_state()

	def stop(self):
		raise NotImplementedError

	def set_vel(self, fwd_speed, turn_speed, exe_time=1):
		raise NotImplementedError

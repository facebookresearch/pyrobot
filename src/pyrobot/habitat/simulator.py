# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from pyrobot.habitat.sim_utils import make_cfg

import habitat_sim

class HabitatSim(object):
	"""Class that interfaces with Habitat sim"""
	def __init__(self, config): #TODO: extend the arguments of constructor
		self.sim_config = config.SIMULATOR
		self.sim = habitat_sim.Simulator(make_cfg(self.sim_config))

	def get_agents():
		''' Return a list of anget objects'''
		return self.sim.agents

	def get_sensors():
		'''Returns a dictionary of sensorid's and sensors'''
		return self.sim._sensors

	def hi(self):
		print('#################################')

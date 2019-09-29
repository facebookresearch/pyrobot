# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from pyrobot.habitat.sim_utils import make_cfg

import habitat_sim

class HabitatSim(object):
	"""Class that interfaces with Habitat sim"""
	def __init__(self, arg):
		super(HabitatSim, self).__init__()
		self.arg = arg

	def hi(self):
		print('#################################')

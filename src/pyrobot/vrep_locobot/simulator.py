# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import copy
from pyrep import PyRep


class VrepSim(object):
    """Class that interfaces with Habitat sim"""

    def __init__(
        self, configs, scene_path=None, seed=0
    ):  # TODO: extend the arguments of constructor
        self.sim_config = copy.deepcopy(configs.COMMON.SIMULATOR)

        if scene_path is None:
            raise RuntimeError("Please specify the .ttt v-rep scene file path!")
        self.sim = PyRep()
        self.sim.launch(scene_path, headless=False)
        self.sim.start()
        [self.sim.step() for _ in range(50)]

    def __del__(self):
        self.sim.stop()
        self.sim.shutdown()

    def reset(self):
        """Reset the Habitat simultor"""
        raise NotImplemented("Reset method for v-rep has not been implemented")

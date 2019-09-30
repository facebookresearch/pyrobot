# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pyrobot.utils.util as prutil
from pyrobot.core import Camera

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
    import cv2
sys.path.append(ros_path)
from cv_bridge import CvBridge, CvBridgeError


class SimpleCamera(Camera):
	"""docstring for SimpleCamera"""
	def __init__(self, configs, simulator):
		super(SimpleCamera, self).__init__(configs)
		self.sim = simulator.sim

	def get_rgb(self):
		observations = self.sim.get_sensor_observations()
		return observations["rgb"]

	def get_depth(self):
		observations = self.sim.get_sensor_observations()
		return observations["depth"]
		
	def get_rgb_depth(self):
		observations = self.sim.get_sensor_observations()
		return  observations["rgb"], observations["depth"]

	#TODO: Add fucntions for camera instrinsics, pcd, 3d points etc
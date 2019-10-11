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

import habitat_sim.agent as habAgent
import habitat_sim.utils as habUtils


class LoCoBotCamera(Camera):
	"""docstring for SimpleCamera"""
	def __init__(self, configs, simulator):
		super(LoCoBotCamera, self).__init__(configs)
		self.sim = simulator.sim
		self.configs = configs
		self.agent = \
			self.sim.get_agent(self.configs.COMMON.SIMULATOR.DEFAULT_AGENT_ID)

		# Depth Image processor


		# Pan and tilt related vairades.
		self.pan = 0.0
		self.tilt = 0.0

	def get_rgb(self):
		observations = self.sim.get_sensor_observations()
		return observations["rgb"]

	def get_depth(self):
		observations = self.sim.get_sensor_observations()
		return observations["depth"]
		
	def get_rgb_depth(self):
		observations = self.sim.get_sensor_observations()
		return  observations["rgb"], observations["depth"]

	def get_intrinsics(self):

		# Todo: Remove this after we fix intrinsics
		raise NotImplementedError
		"""
		Returns the instrinsic matrix of the camera

		:return: the intrinsic matrix (shape: :math:`[3, 3]`)
		:rtype: np.ndarray
		"""
		# fx = self.configs['Camera.fx']
		# fy = self.configs['Camera.fy']
		# cx = self.configs['Camera.cx']
		# cy = self.configs['Camera.cy']
		Itc = np.array([[fx, 0, cx],
                        [0, fy, cy],
                        [0, 0, 1]])
		return Itc

	def pix_to_3dpt(self, rs, cs, in_cam=False):
		"""
		Get the 3D points of the pixels in RGB images.

		:param rs: rows of interest in the RGB image.
		           It can be a list or 1D numpy array
		           which contains the row indices.
		           The default value is None,
		           which means all rows.
		:param cs: columns of interest in the RGB image.
		           It can be a list or 1D numpy array
		           which contains the column indices.
		           The default value is None,
		           which means all columns.
		:param in_cam: return points in camera frame,
		               otherwise, return points in base frame

		:type rs: list or np.ndarray
		:type cs: list or np.ndarray
		:type in_cam: bool

		:returns: tuple (pts, colors)

		          pts: point coordinates in world frame
		          (shape: :math:`[N, 3]`)

		          colors: rgb values for pts_in_cam
		          (shape: :math:`[N, 3]`)

		:rtype: tuple(np.ndarray, np.ndarray)
		"""

		raise NotImplementedError

	def get_current_pcd(self, in_cam=True):
		"""
		Return the point cloud at current time step (one frame only)

		:param in_cam: return points in camera frame,
		               otherwise, return points in base frame

		:type in_cam: bool
		:returns: tuple (pts, colors)

		          pts: point coordinates in world frame (shape: :math:`[N, 3]`)

		          colors: rgb values for pts_in_cam (shape: :math:`[N, 3]`)
		:rtype: tuple(np.ndarray, np.ndarray)
		"""

		raise NotImplementedError

	@property
	def state(self):
		"""
		Return the current pan and tilt joint angles of the robot camera.

		:return:
		        pan_tilt: A list the form [pan angle, tilt angle]
		:rtype: list
		"""
		return self.get_state()

	def get_state(self):
		"""
		Return the current pan and tilt joint angles of the robot camera.

		:return:
		        pan_tilt: A list the form [pan angle, tilt angle]
		:rtype: list
		"""
		return [self.pan, self.tilt]

	def get_pan(self):
		"""
		Return the current pan joint angle of the robot camera.

		:return:
		        pan: Pan joint angle
		:rtype: float
		"""
		return self.pan

	def get_tilt(self):
		"""
		Return the current tilt joint angle of the robot camera.

		:return:
		        tilt: Tilt joint angle
		:rtype: float
		"""
		return self.tilt

	def set_pan(self, pan, wait=True):
		"""
		Sets the pan joint angle to the specified value.

		:param pan: value to be set for pan joint
		:param wait: wait until the pan angle is set to
		             the target angle.

		:type pan: float
		:type wait: bool
		"""

		self.set_pan_tilt(pan, self.tilt)

	def set_tilt(self, tilt, wait=True):
		"""
		Sets the tilt joint angle to the specified value.

		:param tilt: value to be set for the tilt joint
		:param wait: wait until the tilt angle is set to
		             the target angle.

		:type tilt: float
		:type wait: bool
		"""

		self.set_pan_tilt(self.pan, tilt)

	def _compute_relative_pose(self, pan, tilt):

		pan_link = 0.1 #length of pan link
		tilt_link = 0.1 #length of tilt link
		
		sensor_offset_tilt = np.asarray([0.0, 0.0, -1*tilt_link])

		quat_cam_to_pan = habUtils.quat_from_angle_axis(-1 * self.tilt, 
															np.asarray([1.0,0.0,0.0]))

		sensor_offset_pan = habUtils.quat_rotate_vector(quat_cam_to_pan, 
													    sensor_offset_tilt) 
		sensor_offset_pan += np.asarray([0.0, pan_link, 0.0])								

		quat_pan_to_base = habUtils.quat_from_angle_axis(-1*self.pan, 
															np.asarray([0.0,1.0,0.0]))

		sensor_offset_base = habUtils.quat_rotate_vector(quat_pan_to_base, 
													    sensor_offset_pan) 
		sensor_offset_base += np.asarray([0.0, 0.5, 0.1]) # offset w.r.t base

		# translation
		quat = quat_cam_to_pan*quat_pan_to_base		
		return sensor_offset_base, quat.inverse()

	def set_pan_tilt(self, pan, tilt, wait=True):
		"""
		Sets both the pan and tilt joint angles to the specified values.

		:param pan: value to be set for pan joint
		:param tilt: value to be set for the tilt joint
		:param wait: wait until the pan and tilt angles are set to
		             the target angles.

		:type pan: float
		:type tilt: float
		:type wait: bool
		"""
		self.pan = pan
		self.tilt = tilt
		sensor_offset, quat_base_to_sensor = self._compute_relative_pose(pan, tilt)
		cur_state = self.agent.get_state()
		sensor_position = sensor_offset + cur_state.position
		sensor_quat = cur_state.rotation * quat_base_to_sensor
		cur_state.sensor_states["rgb"].position = sensor_position
		cur_state.sensor_states["rgb"].rotation = sensor_quat

		self.agent.set_state(cur_state)

	def reset(self):
		"""
		This function resets the pan and tilt joints by actuating
		them to their home configuration.
		"""
		self.set_pan_tilt(self.configs.CAMERA.RESET_PAN,
							self.configs.CAMERA.RESET_TILT)
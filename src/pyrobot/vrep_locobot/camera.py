# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pyrobot.utils.util as prutil
from pyrobot.core import Camera

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError


from pyrep.objects.vision_sensor import VisionSensor
from pyrep.const import ObjectType, PerspectiveMode, RenderMode
from pyrep.objects.joint import Joint


class LoCoBotCamera(Camera):
    """docstring for SimpleCamera"""

    def __init__(self, configs, simulator):

        self.sim = simulator.sim
        self.rgb_cam = VisionSensor("kinect_rgb")
        self.depth_cam = VisionSensor("kinect_depth")
        self.rgb_cam.set_render_mode(RenderMode.OPENGL3)
        self.depth_cam.set_render_mode(RenderMode.OPENGL3)

        # Pan and tilt related variables.
        self.pan_joint = Joint("LoCoBot_head_pan_joint")
        self.tilt_joint = Joint("LoCoBot_head_tilt_joint")

    def get_rgb(self):

        return self.rgb_cam.capture_rgb()

    def get_depth(self):

        return self.depth_cam.capture_depth()

    def get_rgb_depth(self):

        return self.get_rgb(), self.get_depth()

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
        Itc = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
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
        return [self.get_pan(), self.get_tilt()]

    def get_pan(self):
        """
		Return the current pan joint angle of the robot camera.

		:return:
		        pan: Pan joint angle
		:rtype: float
		"""
        return self.pan_joint.get_joint_position()

    def get_tilt(self):
        """
		Return the current tilt joint angle of the robot camera.

		:return:
		        tilt: Tilt joint angle
		:rtype: float
		"""
        return self.tilt_joint.get_joint_position()

    def set_pan(self, pan, wait=True):
        """
		Sets the pan joint angle to the specified value.

		:param pan: value to be set for pan joint
		:param wait: wait until the pan angle is set to
		             the target angle.

		:type pan: float
		:type wait: bool
		"""

        self.pan_joint.set_joint_position(pan)
        # [self.sim.step() for _ in range(50)]

    def set_tilt(self, tilt, wait=True):
        """
		Sets the tilt joint angle to the specified value.

		:param tilt: value to be set for the tilt joint
		:param wait: wait until the tilt angle is set to
		             the target angle.

		:type tilt: float
		:type wait: bool
		"""

        self.tilt_joint.set_joint_position(tilt)

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

        self.set_pan(pan)
        self.set_tilt(tilt)

    def reset(self):
        """
		This function resets the pan and tilt joints by actuating
		them to their home configuration.
		"""
        self.set_pan_tilt(self.configs.CAMERA.RESET_PAN, self.configs.CAMERA.RESET_TILT)

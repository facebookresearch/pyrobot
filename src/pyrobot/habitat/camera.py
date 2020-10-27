# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pyrobot.utils.util as prutil
from pyrobot.core import Camera

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError

import habitat_sim.agent as habAgent
import habitat_sim.utils as habUtils
from pyrobot.locobot.camera import DepthImgProcessor
from tf.transformations import euler_from_quaternion, euler_from_matrix


class LoCoBotCamera(object):
    """docstring for SimpleCamera"""

    def __init__(self, configs, simulator):
        self.sim = simulator.sim
        self.configs = configs
        self.agent = self.sim.get_agent(self.configs.COMMON.SIMULATOR.DEFAULT_AGENT_ID)

        # Depth Image processor
        self.depth_cam = DepthImgProcessor(
            subsample_pixs=1,
            depth_threshold=(0, 100),
            cfg_filename="realsense_habitat.yaml"
        )

        # Pan and tilt related vairades.
        self.pan = 0.0
        self.tilt = 0.0
        
    def get_rgb(self):
        observations = self.sim.get_sensor_observations()
        return observations["rgb"][:, :, 0:3]

    def get_depth(self):
        observations = self.sim.get_sensor_observations()
        return observations["depth"]

    def get_rgb_depth(self):
        observations = self.sim.get_sensor_observations()
        return observations["rgb"][:, :, 0:3], observations["depth"]

    def get_intrinsics(self):
        """
		Returns the instrinsic matrix of the camera

		:return: the intrinsic matrix (shape: :math:`[3, 3]`)
		:rtype: np.ndarray
		"""
        fx = self.depth_cam.cfg_data['Camera.fx']
        fy = self.depth_cam.cfg_data['Camera.fy']
        cx = self.depth_cam.cfg_data['Camera.cx']
        cy = self.depth_cam.cfg_data['Camera.cy']
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
        rgb_im, depth_im = self.get_rgb_depth()
        pts_in_cam = self.depth_cam.get_pix_3dpt(depth_im=depth_im, rs=rs, cs=cs)
        pts = pts_in_cam[:3, :].T

        colors = rgb_im[rs, cs].reshape(-1, 3)
        if in_cam:
            return pts, colors

        pts = self._cam2pyrobot(pts)
        return pts, colors

    def _cam2pyrobot(self, pts):
        """
        here, points are  given in camera frame
        the thing to do next is to transform the points from camera frame into the
        global frame of pyrobot environment
        This does not translate to what habitat thinks as origin,
        because pyrobot's habitat-reference origin is `self.agent.init_state`
        So, CAMERA frame -> HABITAT frame -> PYROBOT frame (robot base frame)
        :param pts: point coordinates in camera frame
                  (shape: :math:`[N, 3]`)

        :type pts: np.ndarray

        :returns: pts

                  pts: point coordinates in world frame
                  (shape: :math:`[N, 3]`)

        :rtype: np.ndarray
        """

        cur_state = self.agent.get_state()
        cur_sensor_state = cur_state.sensor_states['rgb']
        initial_rotation = cur_state.rotation
        rot_init_rotation = self._rot_matrix(initial_rotation)

        ros_to_habitat_frame = np.array([[0.0, -1.0, 0.0],
                                         [0.0, 0.0, -1.0],
                                         [1.0, 0.0, 0.0]])

        relative_position = cur_sensor_state.position - cur_state.position
        relative_position = rot_init_rotation.T @ relative_position

        cur_rotation = self._rot_matrix(cur_sensor_state.rotation)
        cur_rotation = rot_init_rotation.T @ cur_rotation
        # now do the final transformation and return the points
        pts = np.dot(pts, cur_rotation.T)
        pts = pts - relative_position
        pts = ros_to_habitat_frame.T @ pts.T
        pts = pts.T
        return pts

    def _rot_matrix(self, habitat_quat):
        quat_list = [habitat_quat.x, habitat_quat.y, habitat_quat.z, habitat_quat.w]
        return prutil.quat_to_rot_mat(quat_list)

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
        rgb_im, depth_im = self.get_rgb_depth()
        pcd_in_cam, colors = self.depth_cam.get_pcd_ic(depth_im=depth_im, rgb_im=rgb_im)
        pts = pcd_in_cam[:3, :].T
        if in_cam:
            return pts, colors
        pts = self._cam2pyrobot(pts)
        return pts, colors

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
        pan_link = 0.1  # length of pan link
        tilt_link = 0.1  # length of tilt link

        sensor_offset_tilt = np.asarray([0.0, 0.0, -1 * tilt_link])

        quat_cam_to_pan = habUtils.quat_from_angle_axis(
            -1 * tilt, np.asarray([1.0, 0.0, 0.0])
        )

        sensor_offset_pan = habUtils.quat_rotate_vector(
            quat_cam_to_pan, sensor_offset_tilt
        )
        sensor_offset_pan += np.asarray([0.0, pan_link, 0.0])

        quat_pan_to_base = habUtils.quat_from_angle_axis(
            -1 * pan, np.asarray([0.0, 1.0, 0.0])
        )

        sensor_offset_base = habUtils.quat_rotate_vector(
            quat_pan_to_base, sensor_offset_pan
        )
        sensor_offset_base += np.asarray([0.0, 0.5, 0.1])  # offset w.r.t base

        # translation
        quat = quat_cam_to_pan * quat_pan_to_base
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
        cur_state = self.agent.get_state() # Habitat frame
        sensor_position = cur_state.position + sensor_offset
        sensor_quat = cur_state.rotation * quat_base_to_sensor
        cur_state.sensor_states["rgb"].position = sensor_position
        cur_state.sensor_states["rgb"].rotation = sensor_quat

        self.agent.set_state(cur_state)

    def reset(self):
        """
		This function resets the pan and tilt joints by actuating
		them to their home configuration.
		"""
        self.set_pan_tilt(self.configs.CAMERA.RESET_PAN, self.configs.CAMERA.RESET_TILT)

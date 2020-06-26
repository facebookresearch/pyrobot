# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import rospkg
import threading
import yaml
from copy import deepcopy

import message_filters
import numpy as np
import pyrobot.utils.util as prutil
import rospy

from pyrobot.core import Camera
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf import TransformListener

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError


def constrain_within_range(value, MIN, MAX):
    return min(max(value, MIN), MAX)


def is_within_range(value, MIN, MAX):
    return (value <= MAX) and (value >= MIN)


class SimpleCamera(Camera):
    """
    This is camera class that interfaces with the Realsense
    camera on the locobot and locobot-lite.
    This class does not have the pan and tilt actuation
    capabilities for the camera.
    """

    def __init__(self, configs):
        """
        Constructor of the SimpleCamera class.

        :param configs: Camera specific configuration object

        :type configs: YACS CfgNode
        """
        super(SimpleCamera, self).__init__(configs=configs)
        self._tf_listener = TransformListener()
        depth_threshold = (
            self.configs.BASE.VSLAM.DEPTH_MIN,
            self.configs.BASE.VSLAM.DEPTH_MAX,
        )
        cfg_filename = self.configs.BASE.VSLAM.CFG_FILENAME
        self.depth_cam = DepthImgProcessor(
            subsample_pixs=1, depth_threshold=depth_threshold, cfg_filename=cfg_filename
        )
        self.cam_cf = self.configs.BASE.VSLAM.RGB_CAMERA_CENTER_FRAME
        self.base_f = self.configs.BASE.VSLAM.VSLAM_BASE_FRAME

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
        trans, rot, T = self.get_link_transform(self.cam_cf, self.base_f)
        base2cam_trans = np.array(trans).reshape(-1, 1)
        base2cam_rot = np.array(rot)
        rgb_im, depth_im = self.get_rgb_depth()
        pcd_in_cam, colors = self.depth_cam.get_pcd_ic(depth_im=depth_im, rgb_im=rgb_im)
        pts = pcd_in_cam[:3, :].T
        if in_cam:
            return pts, colors
        pts = np.dot(pts, base2cam_rot.T)
        pts = pts + base2cam_trans.T
        return pts, colors

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
        trans, rot, T = self.get_link_transform(self.cam_cf, self.base_f)
        base2cam_trans = np.array(trans).reshape(-1, 1)
        base2cam_rot = np.array(rot)
        rgb_im, depth_im = self.get_rgb_depth()
        pts_in_cam = self.depth_cam.get_pix_3dpt(depth_im=depth_im, rs=rs, cs=cs)
        pts = pts_in_cam[:3, :].T
        colors = rgb_im[rs, cs].reshape(-1, 3)
        if in_cam:
            return pts, colors
        pts = np.dot(pts, base2cam_rot.T)
        pts = pts + base2cam_trans.T
        return pts, colors

    def get_link_transform(self, src, tgt):
        """
        Returns the latest transformation from the
        target_frame to the source frame,
        i.e., the transform of source frame w.r.t
        target frame. If the returned
        transform is applied to data, it will transform
        data in the source_frame into
        the target_frame

        For more information, please refer to
        http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms

        :param src: source frame
        :param tgt: target frame
        :type src: string
        :type tgt: string

        :returns: tuple(trans, rot, T)

                  trans: translational vector (shape: :math:`[3,]`)

                  rot: rotation matrix (shape: :math:`[3, 3]`)

                  T: transofrmation matrix (shape: :math:`[4, 4]`)
        :rtype: tuple(np.ndarray, np.ndarray, np.ndarray)
        """
        trans, quat = prutil.get_tf_transform(self._tf_listener, tgt, src)
        rot = prutil.quat_to_rot_mat(quat)
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = trans
        return trans, rot, T


class LoCoBotCamera(SimpleCamera):
    """
    This is camera class that interfaces with the Realsense
    camera and the pan and tilt joints on the robot.
    """

    def __init__(self, configs):
        """
        Constructor of the LoCoBotCamera class.

        :param configs: Object containing configurations for camera,
                        pan joint and tilt joint.

        :type configs: YACS CfgNode
        """
        use_camera = rospy.get_param("use_camera", False)
        use_sim = rospy.get_param("use_sim", False)
        use_camera = use_camera or use_sim
        if not use_camera:
            rospy.logwarn(
                "Neither use_camera, nor use_sim, is not set"
                " to True when the LoCoBot driver is launched."
                "You may not be able to command the camera"
                " correctly using PyRobot!!!"
            )
            return
        super(LoCoBotCamera, self).__init__(configs=configs)

        rospy.Subscriber(
            self.configs.ARM.ROSTOPIC_JOINT_STATES,
            JointState,
            self._camera_pose_callback,
        )

        self.set_pan_pub = rospy.Publisher(
            self.configs.CAMERA.ROSTOPIC_SET_PAN, Float64, queue_size=1
        )
        self.set_tilt_pub = rospy.Publisher(
            self.configs.CAMERA.ROSTOPIC_SET_TILT, Float64, queue_size=1
        )
        self.pan = None
        self.tilt = None
        self.tol = 0.01

    def _camera_pose_callback(self, msg):
        if "head_pan_joint" in msg.name:
            pan_id = msg.name.index("head_pan_joint")
            self.pan = msg.position[pan_id]
        if "head_tilt_joint" in msg.name:
            tilt_id = msg.name.index("head_tilt_joint")
            self.tilt = msg.position[tilt_id]

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
        pan = constrain_within_range(
            np.mod(pan + np.pi, 2 * np.pi) - np.pi,
            self.configs.CAMERA.MIN_PAN,
            self.configs.CAMERA.MAX_PAN,
        )
        self.set_pan_pub.publish(pan)
        if wait:
            for i in range(30):
                rospy.sleep(0.1)
                if np.fabs(self.get_pan() - pan) < self.tol:
                    break

    def set_tilt(self, tilt, wait=True):
        """
        Sets the tilt joint angle to the specified value.

        :param tilt: value to be set for the tilt joint
        :param wait: wait until the tilt angle is set to
                     the target angle.

        :type tilt: float
        :type wait: bool
        """
        tilt = constrain_within_range(
            np.mod(tilt + np.pi, 2 * np.pi) - np.pi,
            self.configs.CAMERA.MIN_TILT,
            self.configs.CAMERA.MAX_TILT,
        )
        self.set_tilt_pub.publish(tilt)
        if wait:
            for i in range(30):
                rospy.sleep(0.1)
                if np.fabs(self.get_tilt() - tilt) < self.tol:
                    break

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
        pan = constrain_within_range(
            np.mod(pan + np.pi, 2 * np.pi) - np.pi,
            self.configs.CAMERA.MIN_PAN,
            self.configs.CAMERA.MAX_PAN,
        )
        tilt = constrain_within_range(
            np.mod(tilt + np.pi, 2 * np.pi) - np.pi,
            self.configs.CAMERA.MIN_TILT,
            self.configs.CAMERA.MAX_TILT,
        )
        self.set_pan_pub.publish(pan)
        self.set_tilt_pub.publish(tilt)
        if wait:
            for i in range(30):
                rospy.sleep(0.1)
                if (
                    np.fabs(self.get_pan() - pan) < self.tol
                    and np.fabs(self.get_tilt() - tilt) < self.tol
                ):
                    break

    def reset(self):
        """
        This function resets the pan and tilt joints by actuating
        them to their home configuration.
        """
        self.set_pan_tilt(self.configs.CAMERA.RESET_PAN, self.configs.CAMERA.RESET_TILT)


class DepthImgProcessor:
    """
    This class transforms the depth image and rgb image to point cloud
    """

    def __init__(
        self,
        subsample_pixs=1,
        depth_threshold=(0, 1.5),
        cfg_filename="realsense_d435.yaml",
    ):
        """
        The constructor for :class:`DepthImgProcessor` class.

        :param subsample_pixs: sample rows and columns for the images
        :param depth_threshold: minimum and maximum of valid depth values
        :param cfg_filename: configuration file name for ORB-SLAM2

        :type subsample_pixs: int
        :type depth_threshold: tuple
        :type cfg_filename: string
        """
        assert (type(depth_threshold) is tuple and 0 < len(depth_threshold) < 3) or (
            depth_threshold is None
        )
        self.subsample_pixs = subsample_pixs
        self.depth_threshold = depth_threshold
        self.cfg_data = self.read_cfg(cfg_filename)
        self.intrinsic_mat = self.get_intrinsic()
        self.intrinsic_mat_inv = np.linalg.inv(self.intrinsic_mat)

        img_pixs = np.mgrid[
            0 : self.cfg_data["Camera.height"] : subsample_pixs,
            0 : self.cfg_data["Camera.width"] : subsample_pixs,
        ]
        img_pixs = img_pixs.reshape(2, -1)
        img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
        self.uv_one = np.concatenate((img_pixs, np.ones((1, img_pixs.shape[1]))))
        self.uv_one_in_cam = np.dot(self.intrinsic_mat_inv, self.uv_one)

    def get_pix_3dpt(self, depth_im, rs, cs):
        """
        :param depth_im: depth image (shape: :math:`[H, W]`)
        :param rs: rows of interest. It can be a list or 1D numpy array
                   which contains the row indices. The default value is None,
                   which means all rows.
        :param cs: columns of interest. It can be a list or 1D numpy array
                   which contains the column indices.
                   The default value is None,
                   which means all columns.
        :type depth_im: np.ndarray
        :type rs: list or np.ndarray
        :type cs: list or np.ndarray

        :return: 3D point coordinates of the pixels in
                 camera frame (shape: :math:`[4, N]`)
        :rtype np.ndarray
        """
        pts_in_cam = prutil.pix_to_3dpt(
            depth_im, rs, cs, self.intrinsic_mat, float(self.cfg_data["DepthMapFactor"])
        )
        return pts_in_cam

    def get_pcd_ic(self, depth_im, rgb_im=None):
        """
        Returns the point cloud (filtered by minimum
        and maximum depth threshold)
        in camera's coordinate frame

        :param depth_im: depth image (shape: :math:`[H, W]`)
        :param rgb_im: rgb image (shape: :math:`[H, W, 3]`)

        :type depth_im: np.ndarray
        :type rgb_im: np.ndarray

        :returns: tuple (pts_in_cam, rgb_im)

                  pts_in_cam: point coordinates in
                              camera frame (shape: :math:`[4, N]`)

                  rgb: rgb values for pts_in_cam (shape: :math:`[N, 3]`)
        :rtype tuple(np.ndarray, np.ndarray)
        """
        # pcd in camera from depth
        depth_im = depth_im[0 :: self.subsample_pixs, 0 :: self.subsample_pixs]
        rgb_im = rgb_im[0 :: self.subsample_pixs, 0 :: self.subsample_pixs]
        depth = depth_im.reshape(-1) / float(self.cfg_data["DepthMapFactor"])
        rgb = None
        if rgb_im is not None:
            rgb = rgb_im.reshape(-1, 3)
        if self.depth_threshold is not None:
            valid = depth > self.depth_threshold[0]
            if len(self.depth_threshold) > 1:
                valid = np.logical_and(valid, depth < self.depth_threshold[1])
            uv_one_in_cam = self.uv_one_in_cam[:, valid]
            depth = depth[valid]
            rgb = rgb[valid]
        else:
            uv_one_in_cam = self.uv_one_in_cam
        pts_in_cam = np.multiply(uv_one_in_cam, depth)
        pts_in_cam = np.concatenate(
            (pts_in_cam, np.ones((1, pts_in_cam.shape[1]))), axis=0
        )
        return pts_in_cam, rgb

    def get_pcd_iw(self, pts_in_cam, extrinsic_mat):
        """
        Returns the point cloud in the world coordinate frame

        :param pts_in_cam: point coordinates in
               camera frame (shape: :math:`[4, N]`)
        :param extrinsic_mat: extrinsic matrix for
               the camera (shape: :math:`[4, 4]`)

        :type pts_in_cam: np.ndarray
        :type extrinsic_mat: np.ndarray

        :return: point coordinates in
                 ORB-SLAM2's world frame (shape: :math:`[N, 3]`)
        :rtype: np.ndarray
        """
        # pcd in world
        pts_in_world = np.dot(extrinsic_mat, pts_in_cam)
        pts_in_world = pts_in_world[:3, :].T
        return pts_in_world

    def read_cfg(self, cfg_filename):
        """
        Reads the configuration file

        :param cfg_filename: configuration file name for ORB-SLAM2

        :type cfg_filename: string

        :return: configurations in the configuration file
        :rtype: dict
        """
        rospack = rospkg.RosPack()
        slam_pkg_path = rospack.get_path("orb_slam2_ros")
        cfg_path = os.path.join(slam_pkg_path, "cfg", cfg_filename)
        with open(cfg_path, "r") as f:
            for i in range(1):
                f.readline()
            cfg_data = yaml.load(f, Loader=yaml.FullLoader)
        return cfg_data

    def get_intrinsic(self):
        """
        Returns the instrinsic matrix of the camera

        :return: the intrinsic matrix (shape: :math:`[3, 3]`)
        :rtype: np.ndarray
        """
        fx = self.cfg_data["Camera.fx"]
        fy = self.cfg_data["Camera.fy"]
        cx = self.cfg_data["Camera.cx"]
        cy = self.cfg_data["Camera.cy"]
        Itc = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        return Itc

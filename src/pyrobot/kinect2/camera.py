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
from std_msgs.msg import Float64

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError


class Kinect2Camera(Camera):
    """
    This is camera class that interfaces with the KinectV2 camera
    """

    def __init__(self, configs):
        """
        Constructor of the KinectV2Camera class.

        :param configs: Camera specific configuration object

        :type configs: YACS CfgNode
        """
        super(Kinect2Camera, self).__init__(configs=configs)
        self.DepthMapFactor = float(self.configs.CAMERA.DEPTH_MAP_FACTOR)
        self.intrinsic_mat = None

    def get_current_pcd(self):
        """
        Return the point cloud at current time step (one frame only)

        :returns: tuple (pts, colors)

                  pts: point coordinates in camera frame (shape: :math:`[N, 3]`)

                  colors: rgb values for pts_in_cam (shape: :math:`[N, 3]`)
        :rtype: tuple(np.ndarray, np.ndarray)
        """
        rgb_im, depth_im = self.get_rgb_depth()
        depth = depth_im.reshape(-1) / self.DepthMapFactor
        rgb = rgb_im.reshape(-1, 3)
        if self.intrinsic_mat is None:
            self.intrinsic_mat = self.get_intrinsics()
            self.intrinsic_mat_inv = np.linalg.inv(self.intrinsic_mat)
            # TODO: image height --> rgb_im.shape[0] and width--> rgb_im.shape[1]
            img_pixs = np.mgrid[0 : rgb_im.shape[0] : 1, 0 : rgb_im.shape[1] : 1]
            img_pixs = img_pixs.reshape(2, -1)
            img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
            self.uv_one = np.concatenate((img_pixs, np.ones((1, img_pixs.shape[1]))))
            self.uv_one_in_cam = np.dot(self.intrinsic_mat_inv, self.uv_one)

        pts_in_cam = np.multiply(self.uv_one_in_cam, depth)
        pts_in_cam = np.concatenate(
            (pts_in_cam, np.ones((1, pts_in_cam.shape[1]))), axis=0
        )
        pts = pts_in_cam[:3, :].T
        return pts, rgb

    def pix_to_3dpt(self, rs, cs, reduce="none", k=5):
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
        :param reduce: whether to consider the depth at nearby pixels
                    'none': no neighbour consideration
                    'mean': depth based on the mean of kernel sized k  centered at [rs,cs] 
                    'max': depth based on the max of kernel sized k  centered at [rs,cs] 
                    'min': depth based on the min of kernel sized k  centered at [rs,cs] 
        :param k: kernel size for reduce type['mean', 'max', 'min']
        
        :type rs: list or np.ndarray
        :type cs: list or np.ndarray
        :type reduce: str
        :tyep k: int

        :returns: tuple (pts, colors)

                  pts: point coordinates in world frame
                  (shape: :math:`[N, 3]`)

                  colors: rgb values for pts_in_cam
                  (shape: :math:`[N, 3]`)

        :rtype: tuple(np.ndarray, np.ndarray)
        """
        rgb_im, depth_im = self.get_rgb_depth()
        pts_in_cam = prutil.pix_to_3dpt(
            depth_im, rs, cs, self.get_intrinsics(), self.DepthMapFactor, reduce, k
        )
        pts = pts_in_cam[:3, :].T
        colors = rgb_im[rs, cs].reshape(-1, 3)
        return pts, colors

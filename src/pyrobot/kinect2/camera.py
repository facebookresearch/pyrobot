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

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
    import cv2
sys.path.append(ros_path)
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
        self.cv_bridge = CvBridge()
        self.camera_info_lock = threading.RLock()
        self.camera_img_lock = threading.RLock()
        self.rgb_img = None
        self.depth_img = None
        self.camera_info = None
        self.camera_P = None
        rospy.Subscriber(self.configs.CAMERA.ROSTOPIC_CAMERA_INFO_STREAM,
                         CameraInfo,
                         self._camera_info_callback)

        rgb_topic = self.configs.CAMERA.ROSTOPIC_CAMERA_RGB_STREAM
        self.rgb_sub = message_filters.Subscriber(rgb_topic, Image)
        depth_topic = self.configs.CAMERA.ROSTOPIC_CAMERA_DEPTH_STREAM
        self.depth_sub = message_filters.Subscriber(depth_topic, Image)
        img_subs = [self.rgb_sub, self.depth_sub]
        self.sync = message_filters.ApproximateTimeSynchronizer(img_subs,
                                                                queue_size=10,
                                                                slop=0.2)
        self.sync.registerCallback(self._sync_callback)
        self.DepthMapFactor = float(self.configs.CAMERA.DEPTH_MAP_FACTOR)
        self.intrinsic_mat = None
        
    def _sync_callback(self, rgb, depth):
        self.camera_img_lock.acquire()
        try:
            self.rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")
            self.rgb_img = self.rgb_img[:, :, ::-1]
            self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.camera_img_lock.release()

    def _camera_info_callback(self, msg):
        self.camera_info_lock.acquire()
        self.camera_info = msg
        self.camera_P = np.array(msg.P).reshape((3, 4))
        self.camera_info_lock.release()

    def get_rgb(self):
        '''
        This function returns the RGB image perceived by the camera.

        :rtype: np.ndarray or None
        '''
        self.camera_img_lock.acquire()
        rgb = deepcopy(self.rgb_img)
        self.camera_img_lock.release()
        return rgb

    def get_depth(self):
        '''
        This function returns the depth image perceived by the camera.

        :rtype: np.ndarray or None
        '''
        self.camera_img_lock.acquire()
        depth = deepcopy(self.depth_img)
        self.camera_img_lock.release()
        return depth

    def get_rgb_depth(self):
        '''
        This function returns both the RGB and depth
        images perceived by the camera.

        :rtype: np.ndarray or None
        '''
        self.camera_img_lock.acquire()
        rgb = deepcopy(self.rgb_img)
        depth = deepcopy(self.depth_img)
        self.camera_img_lock.release()
        return rgb, depth

    def get_intrinsics(self):
        """
        This function returns the camera intrinsics.

        :rtype: np.ndarray
        """
        if self.camera_P is None:
            return self.camera_P
        self.camera_info_lock.acquire()
        P = deepcopy(self.camera_P)
        self.camera_info_lock.release()
        return P[:3, :3]

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
            #TODO: image height --> rgb_im.shape[0] and width--> rgb_im.shape[1]
            img_pixs = np.mgrid[0: rgb_im.shape[0]: 1,
                    0: rgb_im.shape[1]: 1]
            img_pixs = img_pixs.reshape(2, -1)
            img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
            self.uv_one = np.concatenate((img_pixs,
                                        np.ones((1, img_pixs.shape[1]))))
            self.uv_one_in_cam = np.dot(self.intrinsic_mat_inv, self.uv_one)
        
        pts_in_cam = np.multiply(self.uv_one_in_cam, depth)
        pts_in_cam = np.concatenate((pts_in_cam,
                                     np.ones((1, pts_in_cam.shape[1]))),
                                    axis=0)
        pts = pts_in_cam[:3, :].T
        return pts, rgb

    def pix_to_3dpt(self, rs, cs, reduce = 'none', k=5):
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
        assert isinstance(rs,
                    int) or isinstance(rs,
                                        list) or isinstance(rs,
                                                            np.ndarray)
        assert isinstance(cs,
                          int) or isinstance(cs,
                                             list) or isinstance(cs,
                                                                 np.ndarray)
        if isinstance(rs, int):
            rs = [rs]
        if isinstance(cs, int):
            cs = [cs]
        if isinstance(rs, np.ndarray):
            rs = rs.flatten()
        if isinstance(cs, np.ndarray):
            cs = cs.flatten()
        rgb_im, depth_im = self.get_rgb_depth()
        R,C,_ = rgb_im.shape
        if reduce == 'none':
            depth_im = depth_im[rs, cs]
        elif reduce == 'mean':
            depth_im = np.array([np.mean(depth_im[max(i-k,0):min(i+k,R), max(j-k,0):min(j+k,C)]) for i,j in zip(rs,cs)])
        elif reduce == 'max':
            depth_im = np.array([np.max(depth_im[max(i-k,0):min(i+k,R), max(j-k,0):min(j+k,C)]) for i,j in zip(rs,cs)])
        elif reduce == 'min':
            depth_im = np.array([np.min(depth_im[max(i-k,0):min(i+k,R), max(j-k,0):min(j+k,C)]) for i,j in zip(rs,cs)])
        else:
            raise ValueError('Invalid reduce name provided, only the following'
                             ' are currently available: [{}, {}, {}, {}]'.format('none','mean', 'max', 'min'))
        #depth_im = depth_im[rs, cs]
        depth = depth_im.reshape(-1) / self.DepthMapFactor
        img_pixs = np.stack((rs, cs)).reshape(2, -1)
        img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
        uv_one = np.concatenate((img_pixs,
                                 np.ones((1, img_pixs.shape[1]))))
        if self.intrinsic_mat is None:
            self.intrinsic_mat = self.get_intrinsics()
            self.intrinsic_mat_inv = np.linalg.inv(self.intrinsic_mat)
        uv_one_in_cam = np.dot(self.intrinsic_mat_inv, uv_one)
        pts_in_cam = np.multiply(uv_one_in_cam, depth)
        pts_in_cam = np.concatenate((pts_in_cam,
                                     np.ones((1, pts_in_cam.shape[1]))),
                                    axis=0)
        pts = pts_in_cam[:3, :].T
        colors = rgb_im[rs, cs].reshape(-1, 3)
        return pts, colors
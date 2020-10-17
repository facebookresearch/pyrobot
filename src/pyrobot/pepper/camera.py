#!/usr/bin/env python
# coding: utf-8

import rospy
import threading
import message_filters
from copy import deepcopy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from pyrobot.core import Camera
from cv_bridge import CvBridge, CvBridgeError


class PepperCamera(Camera):
    """
    This class is the interface with Pepper's cameras (2 RGB cameras and a
    depth camera).
    """

    def __init__(self, configs):
        """
        Constructor of the PepperCamera class.

        :param configs: Camera specific configuration object

        :type configs: YACS CfgNode
        """
        super(PepperCamera, self).__init__(configs=configs)
        self.cv_bridge = CvBridge()
        self.rgb_top_lock = threading.RLock()
        self.rgb_bottom_lock = threading.RLock()
        self.depth_lock = threading.RLock()
        self.rgb_img_top = None
        self.rgb_img_bottom = None
        self.depth_img = None

        rospy.Subscriber(
            self.configs.CAMERA.ROSTOPIC_CAMERA_TOP_STREAM,
            Image,
            self._rgb_top_callback)

        rospy.Subscriber(
            self.configs.CAMERA.ROSTOPIC_CAMERA_BOTTOM_STREAM,
            Image,
            self._rgb_bottom_callback)

        rospy.Subscriber(
            self.configs.CAMERA.ROSTOPIC_CAMERA_DEPTH_STREAM,
            Image,
            self._rgb_depth_callback)

    def _rgb_top_callback(self, rgb_top):
        self.rgb_top_lock.acquire()

        try:
            self.rgb_img_top = self.cv_bridge.imgmsg_to_cv2(
                rgb_top,
                "rgb8")
            self.rgb_img_top = self.rgb_img_top[:, :, ::-1]

        except CvBridgeError as e:
            rospy.logerr(e)

        self.rgb_top_lock.release()

    def _rgb_bottom_callback(self, rgb_bottom):
        self.rgb_bottom_lock.acquire()

        try:
            self.rgb_img_bottom = self.cv_bridge.imgmsg_to_cv2(
                rgb_bottom,
                "rgb8")
            self.rgb_img_bottom = self.rgb_img_bottom[:, :, ::-1]

        except CvBridgeError as e:
            rospy.logerr(e)

        self.rgb_bottom_lock.release()

    def _rgb_depth_callback(self, depth):
        self.depth_lock.acquire()

        try:
            self.depth_img = self.cv_bridge.imgmsg_to_cv2(
                depth,
                "passthrough")

        except CvBridgeError as e:
            rospy.logerr(e)

        self.depth_lock.release()

    def get_rgb(self):
        """
        This function will raise a not implemented exception. To retrieve
        rgb images from the Pepper robot, the get_rgb_top and get_rgb_bottom
        methods should be used.
        """
        raise NotImplementedError('Use get_rgb_top or get_rgb_bottom instead')

    def get_rgb_top(self):
        """
        This function returns the RGB image perceived by the top camera.

        :rtype: np.ndarray or None
        """
        self.rgb_top_lock.acquire()
        rgb_top = deepcopy(self.rgb_img_top)
        self.rgb_top_lock.release()
        return rgb_top

    def get_rgb_bottom(self):
        """
        This function returns the RGB image perceived by the bottom camera.

        :rtype: np.ndarray or None
        """
        self.rgb_bottom_lock.acquire()
        rgb_bottom = deepcopy(self.rgb_img_bottom)
        self.rgb_bottom_lock.release()
        return rgb_bottom

    def get_depth(self):
        """
        This function returns both the RGB and depth
        images perceived by the camera.

        :rtype: np.ndarray or None
        """
        self.depth_lock.acquire()
        depth = deepcopy(self.depth_img)
        self.depth_lock.release()
        return depth

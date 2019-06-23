# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import copy
import os
from functools import partial
from os.path import expanduser

import numpy as np
import rospy
import tf
import tf.transformations
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from nav_msgs.msg import Odometry
from pyrobot.core import Base
from std_msgs.msg import Empty
from bicycle_model import wrap_theta

class BaseSafetyCallbacks(object):
    """
    This class encapsulates and provides interface to bumper, cliff and
    wheeldrop sensors on the base.  It all also trigers necessary callbacks
    when these sensors get tripped.
    """

    def __init__(self, base):
        self.should_stop = False
        self.bumper = False
        self.cliff = False
        self.wheel_drop = False
        self.subscribers = []

        if base == 'create':
            s = rospy.Subscriber(self.configs.BASE.ROSTOPIC_BUMPER, Bumper,
                                 self.bumper_callback_create)
            self.subscribers.append(s)
            s = rospy.Subscriber(self.configs.BASE.ROSTOPIC_CLIFF, Empty,
                                 self.cliff_callback)
            self.subscribers.append(s)
            s = rospy.Subscriber(self.configs.BASE.ROSTOPIC_WHEELDROP, Empty,
                                 self.wheeldrop_callback)
            self.subscribers.append(s)
        else:
            s = rospy.Subscriber(self.configs.BASE.ROSTOPIC_BUMPER,
                                 BumperEvent, self.bumper_callback_kobuki)
            self.subscribers.append(s)
            s = rospy.Subscriber(self.configs.BASE.ROSTOPIC_CLIFF,
                                 CliffEvent, self.cliff_callback)
            self.subscribers.append(s)
            s = rospy.Subscriber(self.configs.BASE.ROSTOPIC_WHEELDROP,
                                 WheelDropEvent, self.wheeldrop_callback)
            self.subscribers.append(s)

    def bumper_callback_create(self, data):
        bumped = data.is_left_pressed or data.is_right_pressed
        to_bump = data.is_light_left or data.is_light_center_left or \
                  data.is_light_center_right or data.is_light_front_right or \
                  data.is_light_right
        if bumped or to_bump:
            if self.bumper is False:
                rospy.loginfo("Bumper Detected")
            self.should_stop = True
            self.bumper = True
        else:
            self.bumper = False

    def cliff_callback(self, data):
        rospy.loginfo("Cliff Detected")
        self.should_stop = True
        self.cliff = True

    def wheeldrop_callback(self, data):
        rospy.loginfo("Drop the contact")
        self.should_stop = True
        self.wheel_drop = True

    def bumper_callback_kobuki(self, date):
        rospy.loginfo("Bumper Detected")
        self.bumper = True
        self.should_stop = True

    def __del__(self):
        # Delete callback on deletion of object.
        for s in self.subscribers:
            s.unregister()


class XYTState(object):
    """
    This class object which can be used used to hold the pose of the base of
    the robot i.e, (x,y, yaw)
    """

    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.theta = 0.
        self.old_theta = 0
        self._state_f = np.array([self.x, self.y, self.theta],
                                 dtype=np.float32).T
        self.update_called = False

    def update(self, x, y, theta):
        """Updates the state being stored by the object."""
        theta = wrap_theta(theta - self.old_theta) + self.old_theta
        self.old_theta = theta
        self.theta = theta
        self.x = x
        self.y = y
        self._state_f = np.array([self.x, self.y, self.theta],
                                 dtype=np.float32).T
        self.update_called = True

    @property
    def state_f(self):
        """Returns the current state as a numpy array."""
        assert (self.update_called), "Odometry callback hasn't been called."
        return self._state_f


class BaseState(BaseSafetyCallbacks):
    def __init__(self, base, build_map, map_img_dir, configs):
        # Set up SLAM, if requested.
        self.configs = configs
        self.build_map = build_map
        if self.build_map:
            self.vslam = VisualSLAM(
                map_img_dir=map_img_dir,
                cam_pose_tp=self.configs.BASE.VSLAM.ROSTOPIC_CAMERA_POSE,
                cam_traj_tp=self.configs.BASE.VSLAM.ROSTOPIC_CAMERA_TRAJ,
                base_frame=self.configs.BASE.VSLAM.VSLAM_BASE_FRAME,
                camera_frame=self.configs.BASE.VSLAM.RGB_CAMERA_CENTER_FRAME,
                occ_map_rate=self.configs.BASE.VSLAM.OCCUPANCY_MAP_RATE,
                z_min=self.configs.BASE.Z_MIN_TRESHOLD_OCC_MAP,
                z_max=self.configs.BASE.Z_MAX_TRESHOLD_OCC_MAP)

        # Setup odometry callback.
        self.state = XYTState()
        s = rospy.Subscriber(configs.BASE.ROSTOPIC_ODOMETRY, Odometry,
                             lambda msg: self._odometry_callback(msg, 'state'))
        self.subscribers = [s]

        BaseSafetyCallbacks.__init__(self, base)

    def _get_odom_state(self):
        """ Returns the base pose in the (x,y, yaw) format as computed from
        Wheel encoder readings."""
        state = self.state.state_f
        return state.tolist()

    def _odometry_callback(self, msg, state_var):
        """Callback function to populate the state object."""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            orientation_list)
        state = copy.deepcopy(getattr(self, state_var))
        state.update(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        setattr(self, state_var, state)

    def __del__(self):
        """Delete callback on deletion of object."""
        for s in self.subscribers:
            s.unregister()
        BaseSafetyCallbacks.__del__(self)


class TurtlebotBase(Base):
    """
    This is a common base class for the locobot and locobot-lite base.
    """

    def __init__(self,
                 configs,
                 map_img_dir=None,
                 base_controller=None,
                 base_planner=None,
                 base=None,
                 ):
        """
        The constructor for LoCoBotBase class.

        :param configs: configurations read from config file
        :param map_img_dir: parent directory of the saved
               RGB images and depth images

        :type configs: YACS CfgNode
        :type map_img_dir: string
        """
        super(TurtlebotBase, self).__init__(configs=configs)
        use_base = rospy.get_param(
            'use_base', True) or rospy.get_param(
            'use_sim', False)
        if not use_base:
            rospy.logwarn(
                'Neither use_base, nor use_sim, is not set to True '
                'when the LoCoBot driver is launched. '
                'You may not be able to command the base '
                'correctly using PyRobot!')
            return
        if base is None:
            base = configs.BASE.BASE_TYPE
        assert (base in ['kobuki', 'create', ]), \
            'BASE should be one of kobuki, create but is {:s}'.format(base)

        if map_img_dir is None:
            map_img_dir = os.path.join(expanduser("~"), '.ros/Imgs')

        self.build_map = rospy.get_param('use_vslam', False)
        self.base_state = BaseState(base, self.build_map, map_img_dir, configs)

        rospy.on_shutdown(self.clean_shutdown)


    def clean_shutdown(self):
        rospy.loginfo("Stop Turtlebot Base")
        self.stop()

    def get_state(self, state_type):
        """
        Returns the requested base pose in the (x,y, yaw)
        format as computed either from
        Wheel encoder readings or Visual-SLAM

        :param state_type: Requested state type. Ex: Odom, SLAM, etc
        :type state_type: string
        :return: pose of the form [x, y, yaw]
        :rtype: list
        """
        if state_type == 'odom':
            return self.base_state._get_odom_state()
        elif state_type == 'vslam':
            assert self.build_map, "Error: Cannot vslam state " \
                                   "without enabling build map feature"
            assert self.build_map, 'build_map was set to False'
            return self.base_state.vslam.base_pose

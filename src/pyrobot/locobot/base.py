# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import copy
import os
from functools import partial
from os.path import expanduser

import numpy as np
import rospy
import tf
import tf.transformations
from ca_msgs.msg import Bumper
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from nav_msgs.msg import Odometry

try:
    from orb_slam2_ros.vslam import VisualSLAM

    USE_ORB_SLAM2 = True
except:
    USE_ORB_SLAM2 = False

from pyrobot.core import Base
from std_msgs.msg import Empty

from pyrobot.locobot.base_control_utils import (
    MoveBasePlanner,
    _get_absolute_pose,
    LocalActionStatus,
    LocalActionServer,
)
from pyrobot.locobot.base_controllers import (
    ProportionalControl,
    ILQRControl,
    MoveBaseControl,
)

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from std_msgs.msg import Empty

from pyrobot.locobot.bicycle_model import wrap_theta
from actionlib_msgs.msg import GoalStatus


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

        if base == "create":
            s = rospy.Subscriber(
                self.configs.BASE.ROSTOPIC_BUMPER, Bumper, self.bumper_callback_create
            )
            self.subscribers.append(s)
            s = rospy.Subscriber(
                self.configs.BASE.ROSTOPIC_CLIFF, Empty, self.cliff_callback
            )
            self.subscribers.append(s)
            s = rospy.Subscriber(
                self.configs.BASE.ROSTOPIC_WHEELDROP, Empty, self.wheeldrop_callback
            )
            self.subscribers.append(s)
        else:
            s = rospy.Subscriber(
                self.configs.BASE.ROSTOPIC_BUMPER,
                BumperEvent,
                self.bumper_callback_kobuki,
            )
            self.subscribers.append(s)
            s = rospy.Subscriber(
                self.configs.BASE.ROSTOPIC_CLIFF, CliffEvent, self.cliff_callback
            )
            self.subscribers.append(s)
            s = rospy.Subscriber(
                self.configs.BASE.ROSTOPIC_WHEELDROP,
                WheelDropEvent,
                self.wheeldrop_callback,
            )
            self.subscribers.append(s)

    def bumper_callback_create(self, data):
        bumped = data.is_left_pressed or data.is_right_pressed
        to_bump = (
            data.is_light_left
            or data.is_light_center_left
            or data.is_light_center_right
            or data.is_light_front_right
            or data.is_light_right
        )
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
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.old_theta = 0
        self._state_f = np.array([self.x, self.y, self.theta], dtype=np.float32).T
        self.update_called = False

    def update(self, x, y, theta):
        """Updates the state being stored by the object."""
        theta = wrap_theta(theta - self.old_theta) + self.old_theta
        self.old_theta = theta
        self.theta = theta
        self.x = x
        self.y = y
        self._state_f = np.array([self.x, self.y, self.theta], dtype=np.float32).T
        self.update_called = True

    @property
    def state_f(self):
        """Returns the current state as a numpy array."""
        assert self.update_called, "Odometry callback hasn't been called."
        return self._state_f


class BaseState(BaseSafetyCallbacks):
    def __init__(self, base, build_map, map_img_dir, configs):
        # Set up SLAM, if requested.
        self.configs = configs
        self.build_map = build_map
        if self.build_map:
            assert USE_ORB_SLAM2, "Error: Failed to import orb_slam2_ros"
            self.vslam = VisualSLAM(
                map_img_dir=map_img_dir,
                cam_pose_tp=self.configs.BASE.VSLAM.ROSTOPIC_CAMERA_POSE,
                cam_traj_tp=self.configs.BASE.VSLAM.ROSTOPIC_CAMERA_TRAJ,
                base_frame=self.configs.BASE.VSLAM.VSLAM_BASE_FRAME,
                camera_frame=self.configs.BASE.VSLAM.RGB_CAMERA_CENTER_FRAME,
                occ_map_rate=self.configs.BASE.VSLAM.OCCUPANCY_MAP_RATE,
                z_min=self.configs.BASE.Z_MIN_TRESHOLD_OCC_MAP,
                z_max=self.configs.BASE.Z_MAX_TRESHOLD_OCC_MAP,
            )

        # Setup odometry callback.
        self.state = XYTState()
        s = rospy.Subscriber(
            configs.BASE.ROSTOPIC_ODOMETRY,
            Odometry,
            lambda msg: self._odometry_callback(msg, "state"),
        )
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
            orientation_q.w,
        ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        state = copy.deepcopy(getattr(self, state_var))
        state.update(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        setattr(self, state_var, state)

    def __del__(self):
        """Delete callback on deletion of object."""
        for s in self.subscribers:
            s.unregister()
        BaseSafetyCallbacks.__del__(self)


class LoCoBotBase(Base):
    """
    This is a common base class for the locobot and locobot-lite base.
    """

    def __init__(
        self,
        configs,
        map_img_dir=None,
        base_controller="ilqr",
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
        super(LoCoBotBase, self).__init__(configs=configs)
        use_base = rospy.get_param("use_base", False) or rospy.get_param(
            "use_sim", False
        )
        if not use_base:
            rospy.logwarn(
                "Neither use_base, nor use_sim, is not set to True "
                "when the LoCoBot driver is launched. "
                "You may not be able to command the base "
                "correctly using PyRobot!"
            )
            return
        if base is None:
            base = configs.BASE.BASE_TYPE
        assert base in [
            "kobuki",
            "create",
        ], "BASE should be one of kobuki, create but is {:s}".format(base)

        if map_img_dir is None:
            map_img_dir = os.path.join(expanduser("~"), ".ros/Imgs")

        self.build_map = rospy.get_param("use_vslam", False)
        self.base_state = BaseState(base, self.build_map, map_img_dir, configs)

        ###################### Action server specific things######################

        self.action_name = "pyrobot/locobot/base/controller_server"
        self.controller_sub = rospy.Subscriber(
            self.action_name, Empty, self._execute_controller,
        )
        self._as = LocalActionServer()
        self.controller_pub = rospy.Publisher(self.action_name, Empty, queue_size=1)
        # class variables shared when communicating between client and server
        self.smooth, self.use_map, self.close_loop = None, None, None
        self.xyt_position = None
        self.xyt_positions = None
        self.controls = None

        self.load_planner(base_planner)
        self.load_controller(base_controller)
        rospy.on_shutdown(self.clean_shutdown)

    # TODO: change this to a planner object.
    # This requires uniformity across all controllers
    def load_planner(self, base_planner=None):

        """
        Method to load a particular planner for Locobot base

        :param base_planner: Name of the planner to be loaded

        :type base_planner: string
        """

        if not self._as.is_available():
            rospy.logwarn(
                "Current planner is use by action server.\
                Please consider using cancel_goal method before calling this method."
            )
            return False

        if base_planner is None:
            base_planner = self.configs.BASE.BASE_PLANNER
        assert base_planner in [
            "movebase",
            "none",
        ], "BASE.[BASE_PLANNER] should be movebase or none."
        if base_planner == "movebase":
            self.planner = MoveBasePlanner(self.configs, self._as)
        elif base_planner == "none":
            # No path planning is done here.
            self.planner = None
        self.base_planner = base_planner

        rospy.sleep(2)

    # TODO: change this to a controller object.
    # This requires uniformity across all controllers
    def load_controller(self, base_controller=None):

        """
        Method to load a particular controller for Locobot base

        :param base_controller: Name of the controller to be loaded

        :type base_controller: string
        """
        if not self._as.is_available():
            rospy.logwarn(
                "Current controller is use by action server.\
                Please consider using cancel_goal method before calling this method."
            )
            return False

        # Set up low-level controllers.
        if base_controller is None:
            base_controller = self.configs.BASE.BASE_CONTROLLER
        assert base_controller in ["proportional", "ilqr", "movebase"], (
            "BASE.BASE_CONTROLLER should be one of proportional, ilqr, "
            "movebase but is {:s}".format(base_controller)
        )
        self.base_controller = base_controller
        if base_controller == "ilqr":
            self.controller = ILQRControl(
                self.base_state, self.ctrl_pub, self.configs, self._as
            )
        elif base_controller == "proportional":
            self.controller = ProportionalControl(
                self.base_state, self.ctrl_pub, self.configs, self._as
            )
        elif base_controller == "movebase":
            self.controller = MoveBaseControl(self.base_state, self.configs, self._as)

        rospy.sleep(2)

    def _execute_controller(self, goal):

        assert self._called_method in [
            "go_to_absolute",
            "track_trajectory",
        ], "Invalid action server method called"
        if self._called_method == "go_to_absolute":
            result = self._go_to_absolute()
        elif self._called_method == "track_trajectory":
            result = self._track_trajectory()

        if self._as.is_disabled():
            print("Locobot base server is disabled")
        elif self._as.is_preempt_requested():
            rospy.logwarn("Current action has been Preempted")
            self._as.set_preempted()
        elif result:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()

    def _track_trajectory(self):

        try:
            if self.base_controller == "ilqr":
                result = self.controller.track_trajectory(
                    self.xyt_states, self.controls, self.close_loop
                )
            else:
                plan_idx = 0
                result = True
                while True:

                    if self._as.is_preempt_requested():
                        rospy.loginfo("Preempted the tracjectory execution")
                        result = False
                        break

                    plan_idx = min(plan_idx, len(self.xyt_states) - 1)
                    point = self.xyt_states[plan_idx]
                    if not self.controller.go_to_absolute(
                        point, close_loop=self.close_loop
                    ):
                        print("hhlhlhlhlhlh")
                        result = False
                        break
                    if plan_idx == len(self.xyt_states) - 1:
                        break
                    plan_idx += self.configs.BASE.TRACKED_POINT
        except AssertionError as error:
            print(error)
            result = False
        except:
            print("Unexpected error encountered during trajectory tracking!")
            result = False

        return result

    def _go_to_absolute(self):

        # try:
        if self.use_map:
            assert self.build_map, (
                "Error: Cannot use map without " "enabling build map feature"
            )
            if self.base_controller == "ilqr":
                goto = partial(
                    self.go_to_relative, close_loop=self.close_loop, smooth=self.smooth,
                )
                result = self.planner.move_to_goal(self.xyt_position, goto)
            elif self.base_controller == "proportional":
                result = self.planner.move_to_goal(
                    self.xyt_position, self.controller.goto
                )
        else:
            result = self.controller.go_to_absolute(
                self.xyt_position, self.close_loop, self.smooth
            )
        
        return result

    def clean_shutdown(self):
        rospy.loginfo("Stopping LoCoBot Base. Waiting for base thread to finish")
        self._as.disable()
        self.stop()
        self.controller_sub.unregister()
        

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
        if state_type == "odom":
            return self.base_state._get_odom_state()
        elif state_type == "vslam":
            assert self.build_map, (
                "Error: Cannot vslam state " "without enabling build map feature"
            )
            assert self.build_map, "build_map was set to False"
            return self.base_state.vslam.base_pose

    def get_plan(self, xyt_position):
        """
        Generates a plan that can take take the robot to given goal state.

        :param xyt_position: The goal state of the form (x,y,t)

        :type xyt_position: list
        """
        plan, status = self.planner.get_plan_absolute(
            xyt_position[0], xyt_position[1], xyt_position[2]
        )
        if not status:
            raise ValueError("Failed to find a valid plan!")
        return self.planner.parse_plan(plan)

    def _wait(self, wait):

        status = self._as.get_state()
        if wait:
            while status != LocalActionStatus.SUCCEEDED:
                if (
                    status == LocalActionStatus.ABORTED
                    or status == LocalActionStatus.PREEMPTED
                    or status == LocalActionStatus.DISABLED
                ):
                    return False
                status = self._as.get_state()
            return True
        else:
            return None

    def go_to_relative(
        self, xyt_position, use_map=False, close_loop=True, smooth=False, wait=True
    ):
        """
        Moves the robot to the robot to given goal state
        relative to its initial pose.

        :param xyt_position: The  relative goal state of the form (x,y,t)
        :param use_map: When set to "True", ensures that controler is
                        using only free space on the map to move the robot.
        :param close_loop: When set to "True", ensures that controler is
                           operating in open loop by taking
                           account of odometry.
        :param smooth: When set to "True", ensures that the
                       motion leading to the goal is a smooth one.
        :param wait: Makes the process wait at this funciton until the execution is 
                       complete

        :type xyt_position: list or np.ndarray
        :type use_map: bool
        :type close_loop: bool
        :type smooth: bool
        :type wait: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool or None
        """
        start_pos = self.base_state.state.state_f.copy()
        goal_pos = _get_absolute_pose(xyt_position, start_pos.ravel())
        return self.go_to_absolute(goal_pos, use_map, close_loop, smooth, wait)

    def go_to_absolute(
        self, xyt_position, use_map=False, close_loop=True, smooth=False, wait=True
    ):
        """
        Moves the robot to the robot to given goal state in the world frame.

        :param xyt_position: The goal state of the form (x,y,t)
                             in the world (map) frame.
        :param use_map: When set to "True", ensures that controler is
                        using only free space on the map to move the robot.
        :param close_loop: When set to "True", ensures that controler
                           is operating in open loop by taking
                           account of odometry.
        :param smooth: When set to "True", ensures that the motion
                       leading to the goal is a smooth one.
        :param wait: Makes the process wait at this funciton until the execution is 
                       complete

        :type xyt_position: list or np.ndarray
        :type use_map: bool
        :type close_loop: bool
        :type smooth: bool
        :type wait: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """

        if not self._as.is_available():
            rospy.logwarn(
                "Base action server already in use by a different goal.\
                           Please consider using cancel_goal method before calling this method."
            )
            return False
        self._as.set_active()  # block action server and set active
        self.xyt_position = np.asarray(xyt_position)
        self.use_map = use_map
        self.smooth = smooth
        self.close_loop = close_loop
        self._called_method = "go_to_absolute"
        self.controller_pub.publish(Empty())

        self._wait(wait)

    def get_last_goal_result(self):

        """
        Returns the status of the commanded goal action.
        None - No last action to report
        ACTIVE = 1
        PREEMPTED  = 2
        SUCCEEDED = 3
        ABORTED = 4
        FREE = 5
        UNKOWN = 6
        PREEMPTING = 7     
        :rtype: LocalActionStatus

        """

        return self._as.get_state()

    def cancel_last_goal(self, stop_robot=True):

        """
        Cancels the last action server command executions.

        :param stop_robot: stops the robot abruptly if enabled.

        :type stop_robot: bool
        """

        self._as.cancel_goal()

        if stop_robot:
            self.stop()

    def track_trajectory(self, states, controls=None, close_loop=True, wait=True):
        """
        State trajectory that the robot should track.

        :param states: sequence of (x,y,t) states that the robot should track.
        :param controls: optionally specify control sequence as well.
        :param close_loop: whether to close loop on the
                           computed control sequence or not.

        :type states: list
        :type controls: list
        :type close_loop: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """

        if len(states) == 0:
            rospy.loginfo("The given trajectory is empty")
            return

        if not self._as.is_available():
            rospy.logwarn(
                "Base action server already in use by a different goal.\
                           Please consider using cancel_goal method before calling this method."
            )
            return False
        self._as.set_active()  # block action server and set active

        self.xyt_states = states
        self.controls = controls
        self.close_loop = close_loop
        self._called_method = "track_trajectory"
        self.controller_pub.publish(Empty())

        self._wait(wait)

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import time
from math import atan2, cos, sin, pi, radians, degrees, sqrt
import numpy as np
import actionlib
import rospy
import copy
from actionlib_msgs.msg import GoalStatusArray, GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from numpy import sign
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import tf
from pyrobot.locobot.base_control_utils import (
    TrajectoryTracker,
    position_control_init_fn,
    _get_absolute_pose,
    SimpleGoalState,
    check_server_client_link,
)
from pyrobot.locobot.base_control_utils import build_pose_msg
from pyrobot.locobot.bicycle_model import BicycleSystem

# if robot rotates by this much amount, then we considered that its moving
# at that speed
ROT_MOVE_THR = radians(1)
# if robot moves by this much amount(in meter), we asuume its moving at
# this speed
LIN_MOVE_THR = 0.01
# if the error in angle(radian) is less than this value, we consider task
# to be done
ROT_ERR_THR = radians(1)
# if the error in position(meter) is less than this value we consider task
# to be done
LIN_ERR_THR = 0.01
VEL_DELTA = 0.01  # increment in the velocity
HZ = 20  # freq at which the controller should run
SLEEP_TIME = 0.1  # sleep time after every action


class ProportionalControl:
    """
    This class encapsulates and provides interface to a Proportional
    controller used to control the base
    """

    def __init__(self, bot_base, ctrl_pub, configs, action_server):
        """
        The constructor for ProportionalControl class.

        :param configs: configurations read from config file
        :param base_state: an object consisting of an
                           instance of BaseState.
        :param ctrl_pub: a ros publisher used to publish velocity
                         commands to base of the robot.

        :type configs: dict
        :type base_state: BaseState
        :type ctrl_pub: rospy.Publisher
        """
        self._as = action_server

        self.configs = configs
        self.bot_base = bot_base

        self.MAP_FRAME = self.configs.BASE.MAP_FRAME
        self.BASE_FRAME = self.configs.BASE.VSLAM.VSLAM_BASE_FRAME

        self.ctrl_pub = ctrl_pub

        self.rot_move_thr = ROT_MOVE_THR
        # threshold for linear.. if its more than this then we consider its
        # moving
        self.lin_move_thr = LIN_MOVE_THR

        self.rot_max_vel = self.configs.BASE.MAX_ABS_TURN_SPEED_P_CONTROLLER
        self.lin_max_vel = self.configs.BASE.MAX_ABS_FWD_SPEED_P_CONTROLLER
        self.translation_treshold = self.configs.BASE.TRANSLATION_TRESHOLD

        # threshold between which if error lies, we think of task being don
        self.rot_error_thr = ROT_ERR_THR
        self.dist_error_thr = LIN_ERR_THR

        self.vel_delta = VEL_DELTA
        self.hz = HZ
        self._transform_listener = TransformListener()
        self.ignore_collisions = False

    def _cmd_vel(self, lin_vel=0.0, rot_vel=0.0):
        # writting vel to robot
        cmd = Twist()
        cmd.angular.z = rot_vel
        cmd.linear.x = lin_vel
        self.ctrl_pub.publish(cmd)

    def stop(self):
        rospy.loginfo("Stopping base!")
        self._cmd_vel(lin_vel=0.0, rot_vel=0.0)
        self.bot_base.should_stop = False

    def _norm_pose(self, data):
        # convert angle to +pi to -pi
        return atan2(sin(data), cos(data))

    def _step_angle(self, action=0.0):
        # target is absolute orientation wrt to current orientation
        # target is in radian
        target = action
        init_pose = self.bot_base.state.theta
        init_err = self._norm_pose(target)
        target_world = self._norm_pose(target + init_pose)

        cmd_vel = 0
        min_vel = 0
        got_min_vel = False
        prev_time = time.time()
        self.bot_base.should_stop = False

        ret_val = True

        while True:

            if self._as.is_preempt_requested():
                rospy.loginfo("Preempted the Proportional execution")

                return False

            if self.bot_base.should_stop:
                if not self.ignore_collisions:
                    rospy.loginfo("curr error = {} meters".format(cur_error))

                    self.stop()
                    return False

            if time.time() - prev_time > (1.0 / self.hz):
                cur_error = self._norm_pose(target_world - self.bot_base.state.theta)

                # stop if error goes beyond some value
                if abs(cur_error) < self.rot_error_thr:
                    rospy.loginfo("Reached goal")
                    rospy.loginfo("curr_error = {} degrees".format(degrees(cur_error)))
                    self._cmd_vel(rot_vel=0.0)
                    break

                # for getting the min velocity at wchich bot starts to move
                if not (got_min_vel) and abs(cur_error - init_err) > self.rot_move_thr:
                    got_min_vel = True
                    min_vel = abs(cmd_vel)

                # doing the linear increse part
                if init_err != 0 and cur_error / init_err > 0.5:
                    if abs(cmd_vel) < self.rot_max_vel:
                        cmd_vel += sign(cur_error) * self.vel_delta

                # elif abs(cur_error) < self.drop_ang:
                else:
                    if abs(cmd_vel) > 0.75 * min_vel:
                        # 0.75 as I found min velocity is always above the
                        # actual min required velocity
                        cmd_vel -= sign(cur_error) * self.vel_delta

                # change the sign of init error if it misses
                if abs(cur_error) > abs(init_err):
                    init_err = cur_error

                # chnage the init error if you overshooot
                if cur_error * init_err < 0:
                    cmd_vel = cmd_vel / 10
                    init_err = cur_error

                self._cmd_vel(rot_vel=cmd_vel)
                prev_time = time.time()

            rospy.sleep(SLEEP_TIME)
        self.bot_base.should_stop = False
        return ret_val

    def _step_x(self, action=0.0):
        # target is the distance in x direction that robot has to move
        # target is in meter(only works for positive )
        target = action
        init_x = self.bot_base.state.x
        init_y = self.bot_base.state.y
        init_err = abs(target)

        cmd_vel = 0
        min_vel = 0
        got_min_vel = False
        prev_time = time.time()
        self.bot_base.should_stop = False
        ret_val = True
        while True:

            if self._as.is_preempt_requested():
                rospy.loginfo("Preempted the Proportional execution")

                return False

            if self.bot_base.should_stop:
                if not self.ignore_collisions:
                    rospy.loginfo("curr error = {} meters".format(cur_error))

                    self.stop()
                    return False

            if time.time() - prev_time > (1.0 / self.hz):
                cur_error = abs(
                    abs(target)
                    - sqrt(
                        (self.bot_base.state.x - init_x) ** 2
                        + (self.bot_base.state.y - init_y) ** 2
                    )
                )

                # stop if error goes beyond some value
                if abs(cur_error) < self.dist_error_thr:
                    rospy.loginfo("Reached goal")
                    rospy.loginfo("curr error = {} meters".format(cur_error))
                    self._cmd_vel(lin_vel=0.0)
                    break

                # for getting the min velocity at wchich bot starts to move
                if not (got_min_vel) and abs(cur_error - init_err) > self.lin_move_thr:
                    got_min_vel = True
                    min_vel = abs(cmd_vel)
                    # rospy.loginfo("min vel = ", min_vel)

                # doing the linear increse part
                if cur_error / init_err > 0.6:
                    if abs(cmd_vel) < self.lin_max_vel:
                        cmd_vel += sign(target) * self.vel_delta

                # elif abs(cur_error) < self.drop_ang:
                else:
                    if abs(cmd_vel) > 0.75 * min_vel:
                        cmd_vel -= sign(target) * self.vel_delta

                # change the sign of init error if it misses
                if abs(cur_error) > abs(init_err):
                    init_err = cur_error
                    target = sign(target) * cur_error

                # chnage the init error if you overshooot
                if cur_error * init_err < 0:
                    cmd_vel = cmd_vel / 10
                    init_err = cur_error
                    target = -sign(target) * cur_error

                self._cmd_vel(lin_vel=cmd_vel)
                prev_time = time.time()
            rospy.sleep(SLEEP_TIME)
        self.bot_base.should_stop = False
        return ret_val

    def goto(self, xyt_position=None):
        """
        Moves the robot to the robot to given goal state in
        the relative frame (base frame).

        :param xyt_position: The goal state of the form
                             (x,y,t) in the relative (base) frame.

        :type xyt_position: list
        """
        if xyt_position is None:
            xyt_position = [0.0, 0.0, 0.0]
        rospy.loginfo("BASE goto, cmd: {}".format(xyt_position))
        x = xyt_position[0]
        y = xyt_position[1]
        rot = xyt_position[2]

        if sqrt(x * x + y * y) < self.translation_treshold:
            self._step_angle(rot)
            return True

        theta_1 = atan2(y, x)
        dist = sqrt(x ** 2 + y ** 2)

        if theta_1 > pi / 2:
            theta_1 = theta_1 - pi
            dist = -dist

        if theta_1 < -pi / 2:
            theta_1 = theta_1 + pi
            dist = -dist

        theta_2 = -theta_1 + rot
        # first rotate by theta1
        if not self._step_angle(theta_1):
            return False
        # move the distance
        if not self._step_x(dist):
            return False
        # second rotate by theta2
        if not self._step_angle(theta_2):
            return False

        return True

    def _get_xyt(self, pose):
        """Processes the pose message to get (x,y,theta)"""
        orientation_list = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        goal_position = [pose.pose.position.x, pose.pose.position.y, yaw]
        return goal_position

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=False):
        """
        Moves the robot to the robot to given goal state in
        the world frame using proportional control.

        :param xyt_position: The goal state of the form
                             (x,y,t) in the world (map) frame.
        :param close_loop: When set to "True", ensures that
                           controler is operating in open loop by
                           taking account of odometry.
        :param smooth: When set to "True", ensures that the
                       motion leading to the goal is a smooth one.

        :type xyt_position: list or np.ndarray
        :type close_loop: bool
        :type smooth: bool
        """
        assert (
            not smooth
        ), "Proportional controller \
                        cannot generate smooth motion"
        assert (
            close_loop
        ), "Proportional controller \
                        cannot work in open loop"
        pose_stamped = build_pose_msg(
            xyt_position[0], xyt_position[1], xyt_position[2], self.MAP_FRAME
        )
        base_pose = self._transform_listener.transformPose(
            self.BASE_FRAME, pose_stamped
        )
        return self.goto(self._get_xyt(base_pose))


class ILQRControl(TrajectoryTracker):
    """
    Class to implement LQR based feedback controllers
    on top of mobile bases.
    """

    def __init__(self, bot_base, ctrl_pub, configs, action_server):
        """
        Constructor for ILQR based Control.

        :param bot_base: Object that has necessary variables
                         that capture the robot state,
                         collision checking, etc.
        :param ctrl_pub: Publisher topic to send commands to.
        :param configs: yacs configuration object.
        :type base_state: BaseState
        :type ctrl_pub: rospy.Publisher
        """
        self._as = action_server
        self.configs = configs
        self.max_v = self.configs.BASE.MAX_ABS_FWD_SPEED
        self.min_v = -self.configs.BASE.MAX_ABS_FWD_SPEED
        self.max_w = self.configs.BASE.MAX_ABS_TURN_SPEED
        self.min_w = -self.configs.BASE.MAX_ABS_TURN_SPEED
        self.rate = rospy.Rate(self.configs.BASE.BASE_CONTROL_RATE)
        self.dt = 1.0 / self.configs.BASE.BASE_CONTROL_RATE

        self.ctrl_pub = ctrl_pub

        self.bot_base = bot_base
        self.system = BicycleSystem(
            self.dt, self.min_v, self.max_v, self.min_w, self.max_w
        )

    @property
    def should_stop(self):
        return self.bot_base.should_stop

    @should_stop.setter
    def should_stop(self, value):
        self.bot_base.should_stop = value

    @property
    def state(self):
        return self.bot_base.state

    def go_to_relative(self, xyt_position, close_loop=True, smooth=True):
        """
        Relative pose that the robot should go to.
        """
        start_pos = self.state.state_f.copy()
        goal_pos = _get_absolute_pose(xyt_position, start_pos.ravel())
        return self.go_to_absolute(goal_pos, close_loop, smooth)

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=True):
        """
        Moves the robot to the robot to given goal state in
        the world frame using ILQR control.

        :param xyt_position: The goal state of the form
                             (x,y,t) in the world (map) frame.
        :param close_loop: When set to "True", ensures that
                           controler is operating in open loop by
                           taking account of odometry.
        :param smooth: When set to "True", ensures that the
                       motion leading to the goal is a smooth one.

        :type xyt_position: list or np.ndarray
        :type close_loop: bool
        :type smooth: bool
        """
        start_pos = self.state.state_f.copy()
        goal_pos = copy.deepcopy(xyt_position)
        reverse = self.min_v < 0
        states = self._compute_trajectory_no_map(start_pos, goal_pos, smooth, reverse)
        # Compute and execute the plan.
        return self.track_trajectory(states, close_loop=close_loop)

    def _compute_trajectory_no_map(self, start_pos, goal_pos, smooth, reverse):
        typ = "smooth" if smooth else "sharp"
        init_states = position_control_init_fn(
            typ, start_pos, goal_pos, self.dt, self.max_v, self.max_w, reverse
        )
        return init_states

    def track_trajectory(self, states, controls=None, close_loop=True):
        """
        State trajectory that the robot should track using ILQR control.

        :param states: sequence of (x,y,t) states that the robot should track.
        :param controls: optionally specify control sequence as well.
        :param close_loop: whether to close loop on the
                           computed control sequence or not.

        :type states: list
        :type controls: list
        :type close_loop: bool
        """
        # Compute plan
        plan = self.generate_plan(states, controls)
        if not plan:
            return False
        # Execute a plan
        return self.execute_plan(plan, close_loop)


class MoveBaseControl(object):
    """This class encapsulates and provides interface to move_base controller
    used to control the base
    """

    def __init__(self, base_state, configs, action_server):
        """
        The constructor for MoveBaseControl class.

        :param configs: configurations read from config file
        :param base_state: an object consisting of an instance of BaseState.
        :type configs: dict
        :type base_state: BaseState
        """
        self._as = action_server
        self.configs = configs
        self.base_state = base_state
        self.MAP_FRAME = self.configs.BASE.MAP_FRAME
        self.BASE_FRAME = self.configs.BASE.VSLAM.VSLAM_BASE_FRAME
        self.move_base_sac = actionlib.SimpleActionClient(
            self.configs.BASE.ROSTOPIC_BASE_ACTION_COMMAND, MoveBaseAction
        )

        rospy.Subscriber(
            self.configs.BASE.ROSTOPIC_MOVE_BASE_STATUS,
            GoalStatusArray,
            self._move_base_status_callback,
        )
        self.move_base_cancel_goal_pub = rospy.Publisher(
            self.configs.BASE.ROSTOPIC_GOAL_CANCEL, GoalID, queue_size=1
        )

        self.execution_status = None

    def cancel_goal(self):
        self.move_base_cancel_goal_pub.publish(GoalID())
        self.base_state.should_stop = False

    def _move_base_status_callback(self, msg):
        if len(msg.status_list) > 0:
            self.execution_status = msg.status_list[-1].status

    def _send_action_goal(self, x, y, theta, frame):
        """A function to send the goal state to the move_base action server """
        goal = MoveBaseGoal()
        goal.target_pose = build_pose_msg(x, y, theta, frame)
        goal.target_pose.header.stamp = rospy.Time.now()

        rospy.loginfo("Waiting for the server")
        self.move_base_sac.wait_for_server()

        rospy.loginfo("Sending the goal")
        self.move_base_sac.send_goal(goal)

        rospy.sleep(0.1)
        rospy.loginfo("Waiting for the Result")
        while True:
            if self._as.is_preempt_requested():
                rospy.loginfo("Preempted the Movebase execution")
                self.cancel_goal()
                return False
            if self.execution_status is 4:
                rospy.loginfo("move_base failed to find a valid plan to goal")
                return False
            if self.execution_status is 3:
                rospy.loginfo("Base reached the goal state")
                return True
            if self.base_state.should_stop:
                rospy.loginfo("Base asked to stop. Cancelling goal sent to move_base.")
                self.cancel_goal()
                return False

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=False):
        """
        Moves the robot to the robot to given goal state in the world frame.

        :param xyt_position: The goal state of the form (x,y,t) in the world
                             (map) frame.
        :param close_loop: When set to "True", ensures that controler is
                           operating in open loop by taking account of
                           odometry.
        :param smooth: When set to "True", ensures that the motion leading to
                       the goal is a smooth one.

        :type xyt_position: list or np.ndarray
        :type close_loop: bool
        :type smooth: bool
        """
        assert not smooth, "movebase controller cannot generate smooth motion"
        assert close_loop, "movebase controller cannot work in open loop"
        return self._send_action_goal(
            xyt_position[0], xyt_position[1], xyt_position[2], self.MAP_FRAME
        )


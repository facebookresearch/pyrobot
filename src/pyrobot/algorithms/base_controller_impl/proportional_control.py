from pyrobot.algorithms.base_controller import BaseController
from pyrobot.algorithms.base_localizer import BaseLocalizer

from pyrobot.algorithms.base_controller_impl.base_control_utils import (
    build_pose_msg,
    _get_absolute_pose,
)

from tf import TransformListener
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist

from math import atan2, cos, sin, pi, radians, degrees, sqrt

import rospy
import copy
import time

import numpy as np
from numpy import sign


class ProportionalControl(BaseController):
    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):
        super(BaseController, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )

        self.robot_label = list(self.robots.keys())[0]
        self.bot_base = self.robots[self.robot_label]["base"]

        self.ctrl_pub = self.bot_base.ctrl_pub

        self.rot_max_vel = self.configs.MAX_ABS_TURN_SPEED_P_CONTROLLER
        self.lin_max_vel = self.configs.MAX_ABS_FWD_SPEED_P_CONTROLLER
        self.translation_treshold = self.configs.TRANSLATION_TRESHOLD
        self.rot_error_thr = self.configs.ROT_ERR_THR
        self.dist_error_thr = self.configs.LIN_ERR_THR
        self.rot_move_thr = self.configs.ROT_MOVE_THR
        self.lin_move_thr = self.configs.LIN_MOVE_THR

        self.vel_delta = self.configs.VEL_DELTA
        self.hz = self.configs.HZ
        self.ignore_collisions = self.configs.IGNORE_COLLISIONS
        self.sleep_time = self.configs.SLEEP_TIME

        self.base_frame = self.bot_base.configs.BASE_FRAME

        self._transform_listener = TransformListener()

    def go_to_relative(self, xyt_position, close_loop=True, smooth=False):
        start_pos = self.algorithms["BaseLocalizer"].get_odom_state()
        goal_pos = _get_absolute_pose(xyt_position, start_pos.ravel())
        return self.go_to_absolute(goal_pos, close_loop, smooth)

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=False):
        assert (
            not smooth
        ), "Proportional controller \
                        cannot generate smooth motion"
        assert (
            close_loop
        ), "Proportional controller \
                        cannot work in open loop"
        pose_stamped = build_pose_msg(
            xyt_position[0],
            xyt_position[1],
            xyt_position[2],
            self.bot_base.configs.MAP_FRAME,
        )
        self._transform_listener.waitForTransform(
            self.base_frame,
            self.bot_base.configs.MAP_FRAME,
            rospy.Time(0),
            rospy.Duration(3),
        )
        base_pose = self._transform_listener.transformPose(
            self.base_frame, pose_stamped
        )
        return self.goto(self._get_xyt(base_pose))

    def track_trajectory(self, states, close_loop=True):
        raise NotImplementedError()

    def stop(self):
        rospy.loginfo("Stopping base!")
        self._cmd_vel(lin_vel=0.0, rot_vel=0.0)
        self.bot_base.base_state.should_stop = False

    def check_cfg(self):
        super().check_cfg()

        assert (
            len(self.algorithms.keys()) == 1
        ), "Proportional controller only have one dependency!"
        assert (
            list(self.algorithms.keys())[0] == "BaseLocalizer"
        ), "Proportional controller only depend on BaseLocalizer!"
        assert isinstance(
            self.algorithms["BaseLocalizer"], BaseLocalizer
        ), "BaseLocalizer module needs to extend BaseLocalizer base class!"

        assert "MAX_ABS_TURN_SPEED_P_CONTROLLER" in self.configs.keys()
        assert "MAX_ABS_FWD_SPEED_P_CONTROLLER" in self.configs.keys()
        assert "TRANSLATION_TRESHOLD" in self.configs.keys()
        assert "ROT_ERR_THR" in self.configs.keys()
        assert "LIN_ERR_THR" in self.configs.keys()
        assert "ROT_MOVE_THR" in self.configs.keys()
        assert "LIN_MOVE_THR" in self.configs.keys()
        assert "VEL_DELTA" in self.configs.keys()
        assert "HZ" in self.configs.keys()
        assert "IGNORE_COLLISIONS" in self.configs.keys()
        assert "SLEEP_TIME" in self.configs.keys()

    def _cmd_vel(self, lin_vel=0.0, rot_vel=0.0):
        # writting vel to robot
        cmd = Twist()
        cmd.angular.z = rot_vel
        cmd.linear.x = lin_vel
        self.ctrl_pub.publish(cmd)

    def _norm_pose(self, data):
        # convert angle to +pi to -pi
        return atan2(sin(data), cos(data))

    def _step_angle(self, action=0.0):
        # target is absolute orientation wrt to current orientation
        # target is in radian
        target = action
        init_pose = self.algorithms["BaseLocalizer"].get_odom_state()[2]
        init_err = self._norm_pose(target)
        target_world = self._norm_pose(target + init_pose)

        cmd_vel = 0
        min_vel = 0
        got_min_vel = False
        prev_time = time.time()
        self.bot_base.base_state.should_stop = False

        ret_val = True

        while True:
            if self.bot_base.base_state.should_stop:
                if not self.ignore_collisions:
                    rospy.loginfo("curr error = {} meters".format(cur_error))

                    self.stop()
                    return False

            if time.time() - prev_time > (1.0 / self.hz):
                cur_error = self._norm_pose(
                    target_world - self.algorithms["BaseLocalizer"].get_odom_state()[2]
                )

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

            rospy.sleep(self.sleep_time)
        self.bot_base.base_state.should_stop = False
        return ret_val

    def _step_x(self, action=0.0):
        # target is the distance in x direction that robot has to move
        # target is in meter(only works for positive )
        target = action
        init_x = self.algorithms["BaseLocalizer"].get_odom_state()[0]
        init_y = self.algorithms["BaseLocalizer"].get_odom_state()[1]
        init_err = abs(target)

        cmd_vel = 0
        min_vel = 0
        got_min_vel = False
        prev_time = time.time()
        self.bot_base.base_state.should_stop = False
        ret_val = True
        while True:
            if self.bot_base.base_state.should_stop:
                if not self.ignore_collisions:
                    rospy.loginfo("curr error = {} meters".format(cur_error))

                    self.stop()
                    return False

            if time.time() - prev_time > (1.0 / self.hz):
                cur_error = abs(
                    abs(target)
                    - sqrt(
                        (self.algorithms["BaseLocalizer"].get_odom_state()[0] - init_x)
                        ** 2
                        + (
                            self.algorithms["BaseLocalizer"].get_odom_state()[1]
                            - init_y
                        )
                        ** 2
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
            rospy.sleep(self.sleep_time)
        self.bot_base.base_state.should_stop = False
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

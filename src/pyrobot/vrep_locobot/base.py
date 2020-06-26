import numpy as np
import math
import pyrobot.utils.util as prutil
import rospy
from tf.transformations import euler_from_quaternion, euler_from_matrix

from pyrep.robots.mobiles.nonholonomic_base import NonHolonomicBase
from pyrep.errors import ConfigurationPathError


class LoCoBotBase(object):
    """docstring for SimpleBase"""

    def __init__(self, configs, simulator):
        self.configs = configs
        self.sim = simulator.sim
        self.base = NonHolonomicBase(0, 2, "LoCoBot")
        self.base.set_motor_locked_at_zero_velocity(True)

    def get_full_state(self):
        # Returns habitat_sim.agent.AgentState
        # temp_state = self.agent.get_state()
        # temp_state.position = habUtils.quat_rotate_vector(self._fix_transform().inverse(), temp_state.position)
        # temp_state.rotation = self._fix_transform() * temp_state.rotation
        return self.agent.get_state()

    @property
    def in_collision(self):
        return self.base.assess_collision()

    def get_state(self):
        # Returns (x, y, yaw)
        pose = self.base.get_2d_pose()
        return (pose[0], pose[1], pose[2])

    def stop(self):
        # raise NotImplementedError("Veclocity control is not supported in V-Rep Sim!!")
        self.base.set_joint_target_velocities([0, 0])

    def set_vel(self, fwd_speed, turn_speed, exe_time=1):
        raise NotImplementedError("Veclocity control is not supported in V-Rep Sim!!")

    def go_to_relative(self, xyt_position, use_map=None, close_loop=True, smooth=False):
        """
		Moves the robot to the robot to given
		goal state relative to its initial pose.

		:param xyt_position: The  relative goal state of the form (x,y,t)
		:param use_map: When set to "True", ensures that controler is
		                using only free space on the map to move the robot.
		:param close_loop: When set to "True", ensures that controler is
		                   operating in open loop by
		                   taking account of odometry.
		:param smooth: When set to "True", ensures that the motion
		               leading to the goal is a smooth one.

		:type xyt_position: list
		:type use_map: bool
		:type close_loop: bool
		:type smooth: bool

		:return: True if successful; False otherwise (timeout, etc.)
		:rtype: bool
		"""

        (cur_x, cur_y, cur_yaw) = self.get_state()
        position = [
            cur_x + xyt_position[0],
            cur_y + xyt_position[1],
            cur_yaw + xyt_position[2],
        ]
        return self.go_to_absolute(position, use_map, close_loop, smooth)

    def go_to_absolute(self, xyt_position, use_map=None, close_loop=True, smooth=False):
        """
		Moves the robot to the robot to given goal state in the world frame.

		:param xyt_position: The goal state of the form (x,y,t)
		                     in the world (map) frame.
		:param use_map: When set to "True", ensures that controler is using
		                only free space on the map to move the robot.
		:param close_loop: When set to "True", ensures that controler is
		                   operating in open loop by
		                   taking account of odometry.
		:param smooth: When set to "True", ensures that the motion
		               leading to the goal is a smooth one.

		:type xyt_position: list
		:type use_map: bool
		:type close_loop: bool
		:type smooth: bool

		:return: True if successful; False otherwise (timeout, etc.)
		:rtype: bool
		"""
        if use_map is None:
            use_map = smooth

        try:
            if use_map != smooth:
                raise NotImplementedError(
                    "Use map feature only works when smooth is enables"
                )
        except Exception as error:
            print(error)
            return False

        position = [xyt_position[0], xyt_position[1]]

        try:
            if smooth:
                base_path = self.base.get_nonlinear_path(position, xyt_position[2])
            else:
                base_path = self.base.get_linear_path(position, xyt_position[2])
        except ConfigurationPathError as error:
            print(error)
            return False

        if close_loop:
            self.base.set_2d_pose(xyt_position)
            return True

        done = False
        while not done:
            done = base_path.step()
            self.sim.step()
        self.sim.step()

        return True

    def track_trajectory(self, states, controls, close_loop=True):
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
        raise NotImplementedError

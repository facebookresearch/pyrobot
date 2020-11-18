import numpy as np
import math
import pyrobot.utils.util as prutil
import habitat_sim.agent as habAgent
import habitat_sim.utils as habUtils
from habitat_sim.agent.controls import ActuationSpec
import habitat_sim.errors

import quaternion
import threading
import time
from ..locobot.base_control_utils import LocalActionStatus, LocalActionServer
from .transformations import euler_from_quaternion, euler_from_matrix


class LoCoBotBase(object):
    """docstring for SimpleBase"""

    def __init__(self, configs, simulator):
        self.configs = configs
        self.sim = simulator.sim
        self.agent = self.sim.get_agent(self.configs.COMMON.SIMULATOR.DEFAULT_AGENT_ID)

        self.transform = None
        self.init_state = self.get_full_state()
        self.lin_vel = self.configs.BASE.FWD_SPEED  # m/s
        self.ang_vel = self.configs.BASE.TURN_SPEED  # deg/s
        self.dt = self.configs.BASE.SIM_DT  # sec
        self.collided = False
        self._as = LocalActionServer()

    def execute_action(self, action_name, actuation):
        # actions = "turn_right" or "turn_left" or "move_forward"
        # returns a bool showing if collided or not
        status = self._as.get_state()
        if not status != LocalActionStatus.ACTIVE:
            self._as.set_active()
            self.collided = False
            x = threading.Thread(target=self._act, args=(action_name, actuation, True))
            x.start()
            return True
        else:
            print("Robot is still moving, can't take another move commend")
            return False
        return

    def get_full_state(self):
        # Returns habitat_sim.agent.AgentState
        return self.agent.get_state()

    def _rot_matrix(self, habitat_quat):
        quat_list = [habitat_quat.x, habitat_quat.y, habitat_quat.z, habitat_quat.w]
        return prutil.quat_to_rot_mat(quat_list)

    def get_state(self, state_type="odom"):
        # Returns (x, y, yaw)
        assert state_type == "odom", "Error: Only Odom state is available"
        cur_state = self.get_full_state()

        init_rotation = self._rot_matrix(self.init_state.rotation)

        # true position here refers to the relative position from
        # where `self.init_state` is treated as origin
        true_position = cur_state.position - self.init_state.position
        true_position = np.matmul(init_rotation.transpose(), true_position, dtype=np.float64)

        cur_rotation = self._rot_matrix(cur_state.rotation)
        cur_rotation = np.matmul(init_rotation.transpose(), cur_rotation, dtype=np.float64)

        (r, pitch, yaw) = euler_from_matrix(cur_rotation, axes="sxzy")
        # Habitat has y perpendicular to map where as ROS has z perpendicular
        # to the map. Where as x is same.
        # Here ROS_X = -1 * habitat_z and ROS_Y = -1*habitat_x
        return (-1 * true_position[2], -1 * true_position[0], yaw)

    def stop(self):
        raise NotImplementedError("Veclocity control is not supported in Habitat-Sim!!")

    def set_vel(self, fwd_speed, turn_speed, exe_time=1):
        raise NotImplementedError("Veclocity control is not supported in Habitat-Sim!!")

    def go_to_relative(
        self, xyt_position, use_map=False, close_loop=False, smooth=False, wait=True
    ):
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
        :param wait: Makes the process wait at this funciton until the execution is 
                       complete

		:type xyt_position: list
		:type use_map: bool
		:type close_loop: bool
		:type smooth: bool

		:return: True if successful; False otherwise (timeout, etc.)
		:rtype: bool
		"""

        try:
            if use_map:
                raise NotImplementedError(
                    "Using map feature is not yet supported for Habitat-Sim"
                )
            if close_loop:
                raise NotImplementedError(
                    "Closed-loop postion control is not supported in Habitat-Sim!"
                )
            if smooth:
                raise NotImplementedError(
                    "Smooth position control feature is not yet for Habitat-Sim"
                )
        except Exception as error:
            print(error)
            return False

        (cur_x, cur_y, cur_yaw) = self.get_state()
        abs_yaw = cur_yaw + xyt_position[2]
        robot_state = self._as.get_state()
        if robot_state != LocalActionStatus.ACTIVE:
            self._as.set_active()
            self.collided = False
            if wait:
                return self._go_to_relative_pose(xyt_position[0], xyt_position[1], abs_yaw, wait=True)
            else:
                x = threading.Thread(
                    target=self._go_to_relative_pose, args=(xyt_position[0], xyt_position[1], abs_yaw, wait)
                )
                x.start()
            return True
        else:
            print("Robot is still moving, can't take another move commend")
            return False

    def go_to_absolute(
        self, xyt_position, use_map=False, close_loop=False, smooth=False, wait=True
    ):
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

        try:
            if use_map:
                raise NotImplementedError(
                    "Using map feature is not yet supported for Habitat-Sim"
                )
            if close_loop:
                raise NotImplementedError(
                    "Closed-loop postion control is not supported in Habitat-Sim!"
                )
            if smooth:
                raise NotImplementedError(
                    "Smooth position control feature is not yet for Habitat-Sim"
                )
        except Exception as error:
            print(error)
            return False

        (cur_x, cur_y, cur_yaw) = self.get_state()
        rel_X = xyt_position[0] - cur_x
        rel_Y = xyt_position[1] - cur_y
        abs_yaw = xyt_position[2]
        # convert rel_X & rel_Y from global frame to  current frame
        R = np.array([[np.cos(cur_yaw), np.sin(cur_yaw)], [-np.sin(cur_yaw), np.cos(cur_yaw)]])
        rel_x, rel_y = np.matmul(R, np.array([rel_X, rel_Y]).reshape(-1, 1))
        robot_state = self._as.get_state()
        if robot_state != LocalActionStatus.ACTIVE:
            self._as.set_active()
            self.collided = False
            if wait:
                return self._go_to_relative_pose(rel_x[0], rel_y[0], abs_yaw, wait=True)
            else:
                x = threading.Thread(
                    target=self._go_to_relative_pose, args=(rel_x[0], rel_y[0], abs_yaw, wait), 
                )
                x.start()
        else:
            print("Robot is still moving, can't take another move commend")
            return False

    def _act(self, action_name, actuation, cont_action = True, direct_call=False):
        """Take the action specified by action_id

		:param action_id: ID of the action. Retreives the action from
		    `agent_config.action_space <AgentConfiguration.action_space>`
		:return: Whether or not the action taken resulted in a collision
		"""
        did_collide = False
        if cont_action:
            dist_moved = 0
            prev_dist_moved = 0
            while dist_moved < actuation:
                if "turn" in action_name:
                    vel = self.ang_vel
                else:
                    vel = self.lin_vel
                prev_dist_moved = dist_moved
                dist_moved = min(dist_moved + self.dt * vel, actuation)
                delta_actuation = dist_moved - prev_dist_moved
                act_spec = ActuationSpec(delta_actuation)
                did_collide = self.agent.controls.action(
                    self.agent.scene_node, action_name, act_spec, apply_filter=True
                )
                if did_collide:
                    self.collided = True
                    break
                time.sleep(self.dt)
            if direct_call:
                self._as.set_succeeded()
        else:
            act_spec = ActuationSpec(actuation)
            self.collided = self.agent.controls.action(
                self.agent.scene_node, action_name, act_spec, apply_filter=True
            )
        return self.collided

    def _go_to_relative_pose(self, rel_x, rel_y, abs_yaw, wait=False):
        # clip relative movements beyond 10 micrometer precision
        # this is done to improve determinism, as habitat-sim doesn't
        # seem to precisely move the robot beyond sub milimeter precision anyways
        if abs(rel_x) < 1e-5:
            rel_x = 0
        if abs(rel_y) < 1e-5:
            rel_y = 0

        if math.sqrt(rel_x ** 2 + rel_y ** 2) > 0.0:
            # rotate to point to (x, y) point
            action_name = "turn_left"
            if rel_y < 0.0:
                action_name = "turn_right"

            v1 = np.asarray([1, 0], dtype=np.float64)
            v2 = np.asarray([rel_x, rel_y], dtype=np.float64)
            cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            angle = np.arccos(cosine_angle)

            did_collide = self._act(action_name, math.degrees(angle), cont_action=not(wait))

            if did_collide:
                print("Error: Collision accured while 1st rotating!")
                self._as.set_preempted()
                return False

            # move to (x,y) point
            did_collide = self._act("move_forward", math.sqrt(rel_x ** 2 + rel_y ** 2), cont_action=not(wait))
            if did_collide:
                print("Error: Collision accured while moving straight!")
                self._as.set_preempted()
                return False
        # rotate to match the final yaw!
        (cur_x, cur_y, cur_yaw) = self.get_state()
        rel_yaw = abs_yaw - cur_yaw

        # clip to micro-degree precision to preserve determinism
        if abs(rel_yaw) < 1e-4:
            rel_yaw = 0

        # convert rel yaw from -pi to pi
        rel_yaw = np.arctan2(np.sin(rel_yaw), np.cos(rel_yaw))

        action_name = "turn_left"
        if rel_yaw < 0.0:
            action_name = "turn_right"
            rel_yaw *= -1

        did_collide = self._act(action_name, math.degrees(rel_yaw), cont_action=not(wait))
        if did_collide:
            print("Error: Collision accured while rotating!")
            self._as.set_preempted()
            return False
        self._as.set_succeeded()
        return True

    def track_trajectory(self, states, controls, close_loop):
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

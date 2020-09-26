from pyrobot.algorithms.base_controller import BaseController
from pyrobot.algorithms.base_localizer import BaseLocalizer

from pyrobot.robots.locobot.base_control_utils import check_server_client_link

import actionlib
from actionlib_msgs.msg import GoalStatusArray, GoalID
from geometry_msgs.msg import Twist
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTolerance,
)
from trajectory_msgs.msg import JointTrajectoryPoint

from actionlib_msgs.msg import GoalStatus
import rospy
import copy
import sys

import numpy as np


class GPMPControl(BaseController):
    """base class of Motion Planning algorithms.
    Specifically, forward/inverse kinematics, and jacobian.
    """

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

        self._as = self.bot_base._as

        self.base_state = self.bot_base.base_state
        self.MAP_FRAME = self.bot_base.configs.MAP_FRAME
        self.BASE_FRAME = self.bot_base.configs.BASE_FRAME

        self.use_map = self.configs.USE_MAP
        self.point_idx = self.configs.TRACKED_POINT

        self.gpmp_ctrl_client_ = actionlib.SimpleActionClient(
            self.configs.GPMP_SERVER_NAME,
            FollowJointTrajectoryAction,
        )
        check_server_client_link(self.gpmp_ctrl_client_)

        self.traj_client_ = actionlib.SimpleActionClient(
            self.configs.TURTLEBOT_TRAJ_SERVER_NAME,
            FollowJointTrajectoryAction,
        )
        check_server_client_link(self.traj_client_)

        self.goal_tolerance = self.configs.GOAL_TOLERANCE
        self.exec_time = self.configs.EXEC_TIME

    def go_to_relative(self, xyt_position, close_loop=True, smooth=True):
        start_pos = self.algorithms["BaseLocalizer"].get_odom_state()
        goal_pos = _get_absolute_pose(xyt_position, start_pos.ravel())
        return self.go_to_absolute(goal_pos, close_loop, smooth)

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=True):
        if self.use_map:
            return self._go_to_absolute_with_map(xyt_position, close_loop=True, smooth=True)
        else:
            return self._go_to_absolute(xyt_position, close_loop=True, smooth=True)


    def track_trajectory(self, states, close_loop=True):
        raise NotImplementedError()

    def stop(self):
        rospy.loginfo("Base asked to stop. Cancelling goal sent to GPMP controller.")

        self.base_state.should_stop = False
        if not self.gpmp_ctrl_client_.gh:
            return

        if self.gpmp_ctrl_client_.simple_state != SimpleGoalState.DONE:
            rospy.loginfo("Cancelling GPMP client.")
            self.gpmp_ctrl_client_.cancel_all_goals()
            rospy.loginfo("Cancelling GPMP-Turtlebot_trajectoryclient.")
            self.traj_client_.cancel_all_goals()

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0
            self.bot_base.ctrl_pub.publish(msg)

    def check_cfg(self):
        assert len(self.robots.keys()) == 1, "One Localizer only handle one base!"
        robot_label = list(self.robots.keys())[0]
        assert (
            "base" in self.robots[robot_label].keys()
        ), "base required for base controllers!"

        assert (
            len(self.algorithms.keys()) == 1
        ), "GPMP controller only have one dependency!"
        assert (
            list(self.algorithms.keys())[0] == "BaseLocalizer"
        ), "GPMP controller only depend on BaseLocalizer!"
        assert isinstance(
            self.algorithms["BaseLocalizer"], BaseLocalizer
        ), "BaseLocalizer module needs to extend BaseLocalizer base class!"

        assert "USE_MAP" in self.configs.keys()
        assert "TRACKED_POINT" in self.configs.keys()
        assert "GPMP_SERVER_NAME" in self.configs.keys()
        assert "TURTLEBOT_TRAJ_SERVER_NAME" in self.configs.keys()
        assert "GOAL_TOLERANCE" in self.configs.keys()
        assert "EXEC_TIME" in self.configs.keys()

    def _build_goal_msg(self, pose, vel, tolerance, exec_time):
        traj_ = FollowJointTrajectoryGoal()
        point = JointTrajectoryPoint()

        for j in range(3):
            point.positions.append(pose[j])
            point.velocities.append(vel[j])

        traj_.trajectory.points.append(point)
        traj_.trajectory.header.stamp = rospy.Time.now()

        joint_tolerance = JointTolerance()
        joint_tolerance.position = tolerance
        traj_.goal_tolerance.append(joint_tolerance)
        traj_.goal_time_tolerance = rospy.Duration(exec_time)  # seconds

        return traj_

    def _go_to_absolute(self, xyt_position, close_loop=True, smooth=True):
        assert smooth, "GPMP controller can only generate smooth motion"
        assert close_loop, "GPMP controller cannot work in open loop"
        self.gpmp_ctrl_client_.send_goal(
            self._build_goal_msg(
                xyt_position, [0.0, 0.0, 0.0], self.goal_tolerance, self.exec_time
            )
        )

        status = self.gpmp_ctrl_client_.get_state()
        if wait:
            while status != GoalStatus.SUCCEEDED:
                if self._as.is_preempt_requested():
                    rospy.loginfo("Preempted the GPMP execution")
                    self.cancel_goal()
                    return False

                if self.base_state.should_stop:
                    rospy.loginfo("collision detection. Stopping")
                    self.cancel_goal()
                    return False
                if status == GoalStatus.ABORTED or status == GoalStatus.PREEMPTED:
                    rospy.loginfo("Error in gpmp sever. Stopping")
                    return False
                status = self.gpmp_ctrl_client_.get_state()
            return True
        else:
            return None

    def _go_to_absolute_with_map(self, xyt_position, close_loop=True, smooth=True):
        cur_state = self.algorithms["BaseLocalizer"].get_odom_state()
        g_distance = np.linalg.norm(
            np.asarray([cur_state[0] - xyt_position[0], cur_state[1] - xyt_position[1]])
        )

        while g_distance > self.configs.TRESHOLD_LIN:

            if self._as.is_preempt_requested():
                rospy.loginfo("Preempted the GPMP execution")
                self.cancel_goal()
                return False

            if self.base_state.should_stop:
                self.cancel_goal()
                return False

            plan, plan_status = self.algorithms["BasePlanner"].get_plan_absolute(
                xyt_position[0], xyt_position[1], xyt_position[2]
            )
            if not plan_status:
                rospy.logerr("Failed to find a valid plan!")
                self.cancel_goal()
                return False

            if len(plan) < self.point_idx:
                point = list(xyt_position)
            else:
                point = [
                    plan[self.point_idx - 1].pose.position.x,
                    plan[self.point_idx - 1].pose.position.y,
                    0,
                ]

                orientation_q = plan[self.point_idx - 1].pose.orientation
                orientation_list = [
                    orientation_q.x,
                    orientation_q.y,
                    orientation_q.z,
                    orientation_q.w,
                ]
                (_, _, point[2]) = tf.transformations.euler_from_quaternion(
                    orientation_list
                )

            self._update_goal(point, wait=False)
            print(point)
            cur_state = self.algorithms["BaseLocalizer"].get_odom_state()
            g_distance = np.linalg.norm(
                np.asarray(
                    [cur_state[0] - xyt_position[0], cur_state[1] - xyt_position[1]]
                )
            )

            status = self.gpmp_ctrl_client_.get_state()
            if status == GoalStatus.ABORTED or status == GoalStatus.PREEMPTED:
                rospy.logerr("GPMP controller failed or interrupted!")
                return False
            rospy.sleep(0.2)

        result = self._go_to_absolute(xyt_position, wait=True)

        if result:
            return True
        else:
            return False

    def _update_goal(self, xyt_position, close_loop=True, smooth=True, wait=True):
        """Updates the the goal state while GPMP
        controller is in execution of previous goal"""
        self.gpmp_ctrl_client_.cancel_goal()
        self._go_to_absolute(xyt_position, close_loop, smooth, wait=wait)
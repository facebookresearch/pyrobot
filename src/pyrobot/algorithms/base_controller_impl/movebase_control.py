from pyrobot.algorithms.base_controller import BaseController
from pyrobot.algorithms.base_localizer import BaseLocalizer

from pyrobot.algorithms.base_controller_impl.base_control_utils import build_pose_msg, _get_absolute_pose

import actionlib
from actionlib_msgs.msg import GoalStatusArray, GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy
import copy

import numpy as np


class MovebaseControl(BaseController):
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
        self.move_base_sac = actionlib.SimpleActionClient(
            self.configs.ROSTOPIC_BASE_ACTION_COMMAND, MoveBaseAction
        )

        rospy.Subscriber(
            self.configs.ROSTOPIC_MOVE_BASE_STATUS,
            GoalStatusArray,
            self._move_base_status_callback,
        )
        self.move_base_cancel_goal_pub = rospy.Publisher(
            self.configs.ROSTOPIC_GOAL_CANCEL, GoalID, queue_size=1
        )

        self.execution_status = None

    def go_to_relative(self, xyt_position, close_loop=True, smooth=False):
        start_pos = self.algorithms["BaseLocalizer"].get_odom_state()
        goal_pos = _get_absolute_pose(xyt_position, start_pos.ravel())
        return self.go_to_absolute(goal_pos, close_loop, smooth)

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=False):
        assert not smooth, "movebase controller cannot generate smooth motion"
        assert close_loop, "movebase controller cannot work in open loop"
        return self._send_action_goal(
            xyt_position[0], xyt_position[1], xyt_position[2], self.MAP_FRAME
        )

    def track_trajectory(self, states, close_loop=True):
        raise NotImplementedError()

    def stop(self):
        """
        Stop the base
        """
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.bot_base.ctrl_pub.publish(msg)

    def check_cfg(self):
        assert len(self.robots.keys()) == 1, "One Controoler only handle one base!"
        robot_label = list(self.robots.keys())[0]
        assert (
            "base" in self.robots[robot_label].keys()
        ), "base required for base controllers!"

        assert (
            len(self.algorithms.keys()) == 1
        ), "Movebase controller only have one dependency!"
        assert (
            list(self.algorithms.keys())[0] == "BaseLocalizer"
        ), "Movebase controller only depend on BaseLocalizer!"
        assert isinstance(
            self.algorithms["BaseLocalizer"], BaseLocalizer
        ), "BaseLocalizer module needs to extend BaseLocalizer base class!"

        assert "ROSTOPIC_BASE_ACTION_COMMAND" in self.configs.keys()
        assert "ROSTOPIC_MOVE_BASE_STATUS" in self.configs.keys()
        assert "ROSTOPIC_GOAL_CANCEL" in self.configs.keys()
        assert "ROSTOPIC_MOVE_BASE_GOAL" in self.configs.keys()

    def _cancel_goal(self):
        self.move_base_cancel_goal_pub.publish(GoalID())
        self.base_state.should_stop = False

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
                self._cancel_goal()
                return False
            if self.execution_status is 4:
                rospy.loginfo("move_base failed to find a valid plan to goal")
                return False
            if self.execution_status is 3:
                rospy.loginfo("Base reached the goal state")
                return True
            if self.base_state.should_stop:
                rospy.loginfo("Base asked to stop. Cancelling goal sent to move_base.")
                self._cancel_goal()
                return False

    def _move_base_status_callback(self, msg):
        if len(msg.status_list) > 0:
            self.execution_status = msg.status_list[-1].status


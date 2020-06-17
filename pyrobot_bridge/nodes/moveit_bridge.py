#!/usr/bin/env python

#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import PyKDL as kdl
import numpy as np
import copy
import rospy
import threading
import tf
import actionlib
from geometry_msgs.msg import Twist
from kdl_parser_py.urdf import treeFromParam
from sensor_msgs.msg import JointState
from trac_ik_python import trac_ik

from pyrobot_bridge.srv import *
import moveit_commander


from pyrobot_bridge.msg import (
    MoveitAction,
    MoveitGoal,
)
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal


class MoveitInterface(object):
    """Interfaces moveit tools to pyrobot"""

    def __init__(self):
        self.init_node = False

        rospy.init_node("pyrobot_moveit")

        self.moveit_server_ = actionlib.SimpleActionServer(
            "/pyrobot/moveit_server",
            MoveitAction,
            execute_cb=self.moveit_cb,
            auto_start=False,
        )
        self.moveit_server_.start()

        self._traj_action = actionlib.SimpleActionClient(
            "execute_trajectory", ExecuteTrajectoryAction
        )

        rospy.sleep(0.1)  # Ensures client spins up properly
        rospy.spin()

    def _init_moveit(self):

        """
        Initialize moveit and setup move_group object
        """
        self.moveit_planner = rospy.get_param("pyrobot/moveit_planner")
        moveit_commander.roscpp_initialize(sys.argv)
        mg_name = rospy.get_param("pyrobot/move_group_name")
        self.moveit_group = moveit_commander.MoveGroupCommander(mg_name)
        self.moveit_group.set_planner_id(self.moveit_planner)
        self.scene = moveit_commander.PlanningSceneInterface()

        self.init_node = True

    def _compute_plan(self, target_joint):
        """
        Computes motion plan to achieve desired target_joint
        :param target_joint: list of length #of joints, angles in radians
        :type target_joint: np.ndarray
        :return: Computed motion plan
        :rtype: moveit_msgs.msg.RobotTrajectory
        """
        # TODO Check if target_joint is valid

        if isinstance(target_joint, np.ndarray):
            target_joint = target_joint.tolist()
        self.moveit_group.set_joint_value_target(target_joint)
        rospy.loginfo("Moveit Motion Planning...")
        return self.moveit_group.plan()

    def wait(self):
        status = self._traj_action.get_state()
        while status != GoalStatus.SUCCEEDED:
            if self.moveit_server_.is_preempt_requested():
                rospy.loginfo("Preempted the Moveit execution by PyRobot")
                self._traj_action.cancel_all_goals()
                self.moveit_server_.set_preempted()
                return
            if status == GoalStatus.ABORTED or status == GoalStatus.PREEMPTED:
                rospy.loginfo("Moveit trajectory execution aborted.")
                self.moveit_server_.set_aborted()
                return
            status = self._traj_action.get_state()
        self.moveit_server_.set_succeeded()

    def _set_joint_positions(self, goal):
        try:
            moveit_plan = self._compute_plan(goal.values)

            if len(moveit_plan.joint_trajectory.points) < 1:
                rospy.logwarn("No motion plan found. No execution attempted")
                self.moveit_server_.set_aborted()
                return

            action_req = ExecuteTrajectoryGoal()
            action_req.trajectory = moveit_plan
            self._traj_action.send_goal(action_req)
        except:
            rospy.logerr("PyRobot-Moveit:Unexpected error in move_ee_xyx")
            self.moveit_server_.set_aborted()
            return

        self.wait()

    def _move_ee_xyz(self, goal):
        try:
            (moveit_plan, fraction) = self.moveit_group.compute_cartesian_path(
                goal.waypoints, goal.eef_step, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

            if len(moveit_plan.joint_trajectory.points) < 1:
                rospy.logwarn("No motion plan found. No execution attempted")
                self.moveit_server_.set_aborted()
                return

            action_req = ExecuteTrajectoryGoal()
            action_req.trajectory = moveit_plan
            self._traj_action.send_goal(action_req)
        except:
            rospy.logerr("PyRobot-Moveit:Unexpected error in move_ee_xyx")
            self.moveit_server_.set_aborted()
            return

        self.wait()

    def moveit_cb(self, goal):

        if not self.init_node:
            self._init_moveit()

        if goal.action_type == "set_joint_positions":
            self._set_joint_positions(goal)
        elif goal.action_type == "move_ee_xyz":
            self._move_ee_xyz(goal)
        else:
            rospy.logerr(
                "Invalid PyRobot-Moveit Action Name, {}".format(goal.action_type)
            )
            self.moveit_server_.set_aborted()
            return


if __name__ == "__main__":
    server = MoveitInterface()

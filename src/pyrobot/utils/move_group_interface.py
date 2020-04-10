# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Copyright 2011-2014, Michael Ferguson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import actionlib
from tf.listener import TransformListener
from geometry_msgs.msg import *
from moveit_msgs.srv import (
    GetCartesianPath,
    GetCartesianPathRequest,
    GetCartesianPathResponse,
    GetPositionIK,
    GetPositionIKRequest,
    GetPositionIKResponse,
    GetPositionFK,
    GetPositionFKRequest,
    GetPositionFKResponse,
    GetMotionPlan,
)
from moveit_msgs.msg import (
    MoveGroupAction,
    MoveGroupGoal,
    MoveItErrorCodes,
    ExecuteTrajectoryAction,
    ExecuteTrajectoryGoal,
)
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.msg import MotionPlanRequest, MotionPlanResponse
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == "_":
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def processResult(result):
    """
    This functions runs through the moveit error
    codes and logs the type of error encountered.
    """
    if result.error_code.val == 1:
        return True
    rospy.loginfo(
        "Moveit Failed with error code: "
        + str(moveit_error_dict[result.error_code.val])
    )
    return False


class MoveGroupInterface(object):
    """
    This class lets you interface with the movegroup node. It provides
    the ability to execute the specified trajectory on the robot by
    communicating to the movegroup node using services.
    """

    def __init__(
        self,
        group,
        fixed_frame,
        gripper_frame,
        cart_srv,
        mp_srv,
        listener=None,
        plan_only=False,
    ):

        self._group = group
        self._fixed_frame = fixed_frame
        self._gripper_frame = gripper_frame  # a.k.a end-effector frame
        self._action = actionlib.SimpleActionClient("move_group", MoveGroupAction)
        self._traj_action = actionlib.SimpleActionClient(
            "execute_trajectory", ExecuteTrajectoryAction
        )
        self._cart_service = rospy.ServiceProxy(cart_srv, GetCartesianPath)
        self._mp_service = rospy.ServiceProxy(mp_srv, GetMotionPlan)
        try:
            self._cart_service.wait_for_service(timeout=3)
        except:
            rospy.logerr("Timeout waiting for Cartesian Planning Service!!")

        try:
            self._mp_service.wait_for_service(timeout=3)
        except:
            rospy.logerr("Timeout waiting for Motion Planning Service!!")

        self._action.wait_for_server()
        if listener == None:
            self._listener = TransformListener()
        else:
            self._listener = listener
        self.plan_only = plan_only

        self.planner_id = None
        self.planning_time = 15.0

    def get_move_action(self):
        return self._action

    def moveToJointPosition(
        self, joints, positions, tolerance=0.01, wait=True, **kwargs
    ):
        """
        Move the arm to set of joint position goals

        :param joints: joint names for which the target position
                is specified.
        :param positions: target joint positions
        :param tolerance: allowed tolerance in the final joint positions.
        :param wait: if enabled, makes the fuctions wait until the
            target joint position is reached

        :type joints: list of string element type
        :type positions: list of float element type
        :type tolerance: float
        :type wait: bool
        """

        # Check arguments
        supported_args = (
            "max_velocity_scaling_factor",
            "planner_id",
            "planning_scene_diff",
            "planning_time",
            "plan_only",
            "start_state",
        )
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("moveToJointPosition: unsupported argument: %s", arg)

        # Create goal
        g = MoveGroupGoal()

        # 1. fill in workspace_parameters

        # 2. fill in start_state
        try:
            g.request.start_state = kwargs["start_state"]
        except KeyError:
            g.request.start_state.is_diff = True

        # 3. fill in goal_constraints
        c1 = Constraints()
        for i in range(len(joints)):
            c1.joint_constraints.append(JointConstraint())
            c1.joint_constraints[i].joint_name = joints[i]
            c1.joint_constraints[i].position = positions[i]
            c1.joint_constraints[i].tolerance_above = tolerance
            c1.joint_constraints[i].tolerance_below = tolerance
            c1.joint_constraints[i].weight = 1.0
        g.request.goal_constraints.append(c1)

        # 4. fill in path constraints

        # 5. fill in trajectory constraints

        # 6. fill in planner id
        try:
            g.request.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.request.planner_id = self.planner_id

        # 7. fill in group name
        g.request.group_name = self._group

        # 8. fill in number of planning attempts
        try:
            g.request.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.request.num_planning_attempts = 1

        # 9. fill in allowed planning time
        try:
            g.request.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.request.allowed_planning_time = self.planning_time

        # Fill in velocity scaling factor
        try:
            g.request.max_velocity_scaling_factor = kwargs[
                "max_velocity_scaling_factor"
            ]
        except KeyError:
            pass  # do not fill in at all

        # 10. fill in planning options diff
        try:
            g.planning_options.planning_scene_diff = kwargs["planning_scene_diff"]
        except KeyError:
            g.planning_options.planning_scene_diff.is_diff = True
            g.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 11. fill in planning options plan only
        try:
            g.planning_options.plan_only = kwargs["plan_only"]
        except KeyError:
            g.planning_options.plan_only = self.plan_only

        # 12. fill in other planning options
        g.planning_options.look_around = False
        g.planning_options.replan = False

        # 13. send goal
        self._action.send_goal(g)
        if wait:
            self._action.wait_for_result()
            result = self._action.get_result()
            return processResult(result)
        else:
            rospy.loginfo("Failed while waiting for action result.")
            return False

    def moveToPose(self, pose, tolerance=0.01, wait=True, **kwargs):

        """
        Move the arm, based on a goal pose_stamped for the end effector.

        :param pose: target pose to which we want to move
                            specified link to
        :param tolerance: allowed tolerance in the final joint positions.
        :param wait: if enabled, makes the fuctions wait until the
            target joint position is reached

        :type pose_stamped: ros message object of type PoseStamped
        :type gripper_frame: string
        :type tolerance: float
        :type wait: bool
        """

        # Check arguments
        supported_args = (
            "max_velocity_scaling_factor",
            "planner_id",
            "planning_time",
            "plan_only",
            "start_state",
        )
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("moveToPose: unsupported argument: %s", arg)

        # Create goal
        g = MoveGroupGoal()

        # 1. fill in request workspace_parameters

        # 2. fill in request start_state
        try:
            g.request.start_state = kwargs["start_state"]
        except KeyError:
            g.request.start_state.is_diff = True

        # 3. fill in request goal_constraints
        c1 = Constraints()

        c1.position_constraints.append(PositionConstraint())
        c1.position_constraints[0].header.frame_id = self._fixed_frame
        c1.position_constraints[0].link_name = self._gripper_frame
        # c1.position_constraints[0].target_point_offset
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [tolerance]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self._fixed_frame
        c1.orientation_constraints[0].orientation = pose.orientation
        c1.orientation_constraints[0].link_name = self._gripper_frame
        c1.orientation_constraints[0].absolute_x_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_y_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_z_axis_tolerance = tolerance
        c1.orientation_constraints[0].weight = 1.0

        g.request.goal_constraints.append(c1)

        # 4. fill in request path constraints

        # 5. fill in request trajectory constraints

        # 6. fill in request planner id
        try:
            g.request.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.request.planner_id = self.planner_id

        # 7. fill in request group name
        g.request.group_name = self._group

        # 8. fill in request number of planning attempts
        try:
            g.request.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.request.num_planning_attempts = 1

        # 9. fill in request allowed planning time
        try:
            g.request.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.request.allowed_planning_time = self.planning_time

        # Fill in velocity scaling factor
        try:
            g.request.max_velocity_scaling_factor = kwargs[
                "max_velocity_scaling_factor"
            ]
        except KeyError:
            pass  # do not fill in at all

        # 10. fill in planning options diff
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 11. fill in planning options plan only
        try:
            g.planning_options.plan_only = kwargs["plan_only"]
        except KeyError:
            g.planning_options.plan_only = self.plan_only

        # 12. fill in other planning options
        g.planning_options.look_around = False
        g.planning_options.replan = False

        # 13. send goal
        self._action.send_goal(g)
        if wait:
            self._action.wait_for_result()
            result = self._action.get_result()
            return processResult(result)
        else:
            rospy.loginfo("Failed while waiting for action result.")
            return False

    def followCartesian(
        self,
        way_points,
        way_point_frame,
        max_step,
        jump_threshold=0,
        link_name=None,  # usually it is Gripper Frame
        start_state=None,  # of type moveit robotstate
        avoid_collisions=True,
    ):
        """
        Movegroup-based cartesian path Control.

        :param way_points: waypoints that the robot needs to track
        :param way_point_frame: the frame in which the waypoints are given.
        :param max_step: resolution (m) of the interpolation
                        on the cartesian path
        :param jump_treshold: a distance in joint space that, if exceeded between 
                    consecutive points, is interpreted as a jump in IK solutions.
        :param link_name: frame or link name for which cartesian trajectory 
                        should be followed
        :param start_state: robot start state of cartesian trajectory
        :param avoid_collisions: if enabled, produces collision free cartesian
                                path

        :type way_points: list of ros message objests of type "Pose"
        :type way_point_frame: string
        :type max_step: float
        :type jump_threshold: float
        :type link_name: string
        :type start_state: moveit_msgs/RobotState
        :type avoid_collisions: bool
        """
        req = GetCartesianPathRequest()
        req.header.stamp = rospy.Time.now()
        req.header.frame_id = way_point_frame
        req.group_name = self._group
        req.waypoints = way_points
        req.max_step = max_step
        req.jump_threshold = jump_threshold
        req.avoid_collisions = avoid_collisions

        if start_state is None:
            req.start_state.is_diff = True
        else:
            req.start_state = start_state

        if link_name is not None:
            req.link_name = link_name

        result = self._cart_service(req)
        rospy.loginfo("Cartesian plan for %f fraction of path", result.fraction)

        if len(result.solution.joint_trajectory.points) < 1:
            rospy.logwarn("No motion plan found. No execution attempted")
            return False

        rospy.loginfo("Executing Cartesian Plan...")

        # 13. send Trajectory
        action_req = ExecuteTrajectoryGoal()
        action_req.trajectory = result.solution
        self._traj_action.send_goal(action_req)
        try:
            self._traj_action.wait_for_result()
            result = self._traj_action.get_result()
            return processResult(result)
        except:
            rospy.logerr("Failed while waiting for action result.")
            return False

    def motionPlanToJointPosition(
        self, joints, positions, tolerance=0.01, wait=True, **kwargs
    ):
        """
        Move the arm to set of joint position goals

        :param joints: joint names for which the target position
                is specified.
        :param positions: target joint positions
        :param tolerance: allowed tolerance in the final joint positions.
        :param wait: if enabled, makes the fuctions wait until the
            target joint position is reached

        :type joints: list of string element type
        :type positions: list of float element type
        :type tolerance: float
        :type wait: bool
        """

        # Check arguments
        supported_args = (
            "max_velocity_scaling_factor",
            "max_acceleration_scaling_factor",
            "planner_id",
            "planning_scene_diff",
            "planning_time",
            "plan_only",
            "start_state",
        )
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("motionPlanToPose: unsupported argument: %s", arg)

        # Create goal
        g = MotionPlanRequest()

        # 1. fill in request workspace_parameters

        # 2. fill in request start_state
        try:
            g.start_state = kwargs["start_state"]
        except KeyError:
            g.start_state.is_diff = True

        # 3. fill in request goal_constraints
        c1 = Constraints()
        for i in range(len(joints)):
            c1.joint_constraints.append(JointConstraint())
            c1.joint_constraints[i].joint_name = joints[i]
            c1.joint_constraints[i].position = positions[i]
            c1.joint_constraints[i].tolerance_above = tolerance
            c1.joint_constraints[i].tolerance_below = tolerance
            c1.joint_constraints[i].weight = 1.0

        g.goal_constraints.append(c1)

        # 4. fill in request path constraints

        # 5. fill in request trajectory constraints

        # 6. fill in request planner id
        try:
            g.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.planner_id = self.planner_id

        # 7. fill in request group name
        g.group_name = self._group

        # 8. fill in request number of planning attempts
        try:
            g.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.num_planning_attempts = 1

        # 9. fill in request allowed planning time
        try:
            g.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.allowed_planning_time = self.planning_time

        # 10. Fill in velocity scaling factor
        try:
            g.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        # 11. Fill in acceleration scaling factor
        try:
            g.max_velocity_scaling_factor = kwargs["max_acceleration_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        result = self._mp_service(g)
        traj = result.motion_plan_response.trajectory.joint_trajectory.points
        if len(traj) < 1:
            rospy.logwarn("No motion plan found.")
            return None
        return traj

    def motionPlanToPose(self, pose, tolerance=0.01, wait=True, **kwargs):
        """
        Move the arm to set of joint position goals

        :param joints: joint names for which the target position
                is specified.
        :param positions: target joint positions
        :param tolerance: allowed tolerance in the final joint positions.
        :param wait: if enabled, makes the fuctions wait until the
            target joint position is reached

        :type joints: list of string element type
        :type positions: list of float element type
        :type tolerance: float
        :type wait: bool
        """

        # Check arguments
        supported_args = (
            "max_velocity_scaling_factor",
            "max_acceleration_scaling_factor",
            "planner_id",
            "planning_scene_diff",
            "planning_time",
            "plan_only",
            "start_state",
        )
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("motionPlanToPose: unsupported argument: %s", arg)

        # Create goal
        g = MotionPlanRequest()

        # 1. fill in request workspace_parameters

        # 2. fill in request start_state
        try:
            g.start_state = kwargs["start_state"]
        except KeyError:
            g.start_state.is_diff = True

        # 3. fill in request goal_constraints
        c1 = Constraints()

        c1.position_constraints.append(PositionConstraint())
        c1.position_constraints[0].header.frame_id = self._fixed_frame
        c1.position_constraints[0].link_name = self._gripper_frame
        # c1.position_constraints[0].target_point_offset
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [tolerance]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self._fixed_frame
        c1.orientation_constraints[0].orientation = pose.orientation
        c1.orientation_constraints[0].link_name = self._gripper_frame
        c1.orientation_constraints[0].absolute_x_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_y_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_z_axis_tolerance = tolerance
        c1.orientation_constraints[0].weight = 1.0

        g.goal_constraints.append(c1)

        # 4. fill in request path constraints

        # 5. fill in request trajectory constraints

        # 6. fill in request planner id
        try:
            g.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.planner_id = self.planner_id

        # 7. fill in request group name
        g.group_name = self._group

        # 8. fill in request number of planning attempts
        try:
            g.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.num_planning_attempts = 1

        # 9. fill in request allowed planning time
        try:
            g.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.allowed_planning_time = self.planning_time

        # 10. Fill in velocity scaling factor
        try:
            g.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        # 11. Fill in acceleration scaling factor
        try:
            g.max_velocity_scaling_factor = kwargs["max_acceleration_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        result = self._mp_service(g)
        traj = result.motion_plan_response.trajectory.joint_trajectory.points
        if len(traj) < 1:
            rospy.logwarn("No motion plan found.")
            return None
        return traj

    def setPlannerId(self, planner_id):
        """
        Sets the planner_id used for all future planning requests.
        :param planner_id: The string for the planner id, set to None to clear
        """
        self.planner_id = str(planner_id)

    def setPlanningTime(self, time):
        """
        Set default planning time to be used for future planning request.
        """
        self.planning_time = time

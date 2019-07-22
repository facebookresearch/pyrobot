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
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest, GetCartesianPathResponse, \
                            GetPositionIK, GetPositionIKRequest, GetPositionIKResponse,\
                            GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes,\
                            ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

def processResult(result):

    if result.error_code.val == 1:
        return True

    rospy.loginfo("Moveit Failed with error code: " + str(moveit_error_dict[result.error_code.val]))
    return False







class MoveGroupInterface(object):

    
    def __init__(self, 
                group, 
                fixed_frame,
                gripper_frame, 
                cart_srv,
                listener=None, 
                plan_only=False):

        self._group = group
        self._fixed_frame = fixed_frame
        self._gripper_frame = gripper_frame # a.k.a end-effector frame
        self._action = actionlib.SimpleActionClient('move_group', # TODO, change this to config
                                                    MoveGroupAction)
        self._traj_action = actionlib.SimpleActionClient('execute_trajectory', # TODO: change this to config
                                                    ExecuteTrajectoryAction)
        self._cart_service =  rospy.ServiceProxy(cart_srv, GetCartesianPath)
        try:
            self._cart_service.wait_for_service(timeout=3)
        except:
            rospy.logerr("Timeout waiting for Cartesian Planning Service!!") 
        

        self._action.wait_for_server()
        if listener == None:
            self._listener = TransformListener()
        else:
            self._listener = listener
        self.plan_only = plan_only

        # TODO KALYAN, figureout a way to set these!
        self.planner_id = None
        self.planning_time = 15.0
        
        

    def _init_kinematics(self, joint_state_topic):

        self.cur_joint_st = None
        self.js_sub = rospy.Subscriber(joint_state_topic,
                                       JointState,
                                       self._js_cb,
                                       queue_size=1)

        ## Forward Kinematics
        self._fk_srv = rospy.ServiceProxy('/compute_fk', #TODO kalyan change this to config
                                         GetPositionFK)
        try:
            self._fk_srv.wait_for_service(timeout=3)
        except:
            rospy.logerr("Timeout waiting for Forward Kinematics Service!!") 

        ## Inverse Kinematics
        self.ik_attempts = 0
        self.ik_timeout = 1.0
        self.avoid_collisions=False

        self._ik_srv = rospy.ServiceProxy('/compute_ik', # TODO kalyan change this to config
                                         GetPositionIK)
        try:
            self._ik_srv.wait_for_service(timeout=3)
        except:
            rospy.logerr("Timeout waiting for Inverse Kinematics Service!!")         


    def _js_cb(self, msg):
        self.cur_joint_st = msg


    def get_current_fk(self):
        while not rospy.is_shutdown() and self.cur_joint_st is None:
            rospy.logwarn("Waiting for a /joint_states message...")
            rospy.sleep(0.1)
        return self.get_fk(self.cur_joint_st)

    def get_fk(self, joint_state, fk_link=None, frame_id=None):
        """
        Do an FK call to with.
        :param sensor_msgs/JointState joint_state: JointState message
            containing the full state of the robot.
        :param str or None fk_link: link to compute the forward kinematics for.
        :param str or None frame_id: Frame in which pose is returned.
        """
        if fk_link is None:
            fk_link = self._gripper_frame

        if frame_id is None:
            frame_id = self._fixed_frame

        req = GetPositionFKRequest()
        req.header.frame_id = frame_id
        req.fk_link_names = [fk_link]
        req.robot_state.joint_state = joint_state
        try:
            resp = self._fk_srv.call(req)
            if resp is not None and len(resp.pose_stamped) >= 1:
                return resp.pose_stamped[0]
            else:
                return None
        except rospy.ServiceException as e:
            rospy.logerr("FK Service exception: " + str(e))
            return None

    def get_ik(self, pose_stamped, use_cur_seed=False):
        if use_cur_seed is True:
            while not rospy.is_shutdown() and self.cur_joint_st is None:
                rospy.logwarn("Waiting for a /joint_states message...")
                rospy.sleep(0.1)
            resp = self._get_ik(pose_stamped=pose_stamped, seed=self.cur_joint_st)
        else:
            resp = self._get_ik(pose_stamped=pose_stamped, seed=None)

        # print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        # print(resp.error_code.val)
        # print(resp.error_code.val == -31)
        # print(resp is None)

        # print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        if resp is None or resp.error_code.val == -31: #-31 is NO IK solution
            rospy.logwarn("IK failed to find a valid solution!")
            return None
        return resp.solution.joint_state



    def _get_ik(self, 
                pose_stamped,
                seed=None,
                group=None,
                ik_timeout=None,
                ik_attempts=None,
                avoid_collisions=None):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        :param str group: The MoveIt! group.
        :param float ik_timeout: The timeout for the IK call.
        :param int ik_attemps: The maximum # of attemps for the IK.
        :param bool avoid_collisions: If to compute collision aware IK.
        """
        if group is None:
            group = self._group
        if ik_timeout is None:
            ik_timeout = self.ik_timeout
        if ik_attempts is None:
            ik_attempts = self.ik_attempts
        if avoid_collisions is None:
            avoid_collisions = self.avoid_collisions
        req = GetPositionIKRequest()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(ik_timeout)
        req.ik_request.attempts = ik_attempts
        req.ik_request.avoid_collisions = avoid_collisions
        #req.ik_request.ik_link_name = self._gripper_frame

        
        #req.ik_request.robot_state.is_diff = True
        if seed is not None:
            req.ik_request.robot_state.joint_state = seed
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print(req)
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        try:
            resp = self._ik_srv.call(req)

            print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
            print(resp)
            print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("IK Service exception: " + str(e))
            return None





    def get_move_action(self):
        return self._action

    ## @brief Move the arm to set of joint position goals
    def moveToJointPosition(self,
                            joints,
                            positions,
                            tolerance=0.01,
                            wait=True,
                            **kwargs):
        # Check arguments
        supported_args = ("max_velocity_scaling_factor",
                          "planner_id",
                          "planning_scene_diff",
                          "planning_time",
                          "plan_only",
                          "start_state")
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("moveToJointPosition: unsupported argument: %s",
                              arg)

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
            g.request.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
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
            rospy.loginfo('Failed while waiting for action result.')
            return False

    ## @brief Move the arm, based on a goal pose_stamped for the end effector.
    def moveToPose(self,
                   pose_stamped,
                   gripper_frame,
                   tolerance=0.01,
                   wait=True,
                   **kwargs):
        # Check arguments
        supported_args = ("max_velocity_scaling_factor",
                          "planner_id",
                          "planning_time",
                          "plan_only",
                          "start_state")
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("moveToPose: unsupported argument: %s",
                              arg)

        # Create goal
        g = MoveGroupGoal()
        pose_transformed = self._listener.transformPose(self._fixed_frame, pose_stamped)

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
        c1.position_constraints[0].link_name = gripper_frame
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [tolerance * tolerance]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(pose_transformed.pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self._fixed_frame
        c1.orientation_constraints[0].orientation = pose_transformed.pose.orientation
        c1.orientation_constraints[0].link_name = gripper_frame
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
            g.request.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
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
            rospy.loginfo('Failed while waiting for action result.')
            return False


    def followCartesian(self, 
                        way_points, 
                        way_point_frame,
                        max_step, 
                        jump_threshold=0,
                        link_name=None,  #usually it is Gripper Frame
                        start_state=None, #of type moveit robotstate
                        avoid_collisions=True):



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
            rospy.logwarn('No motion plan found. No execution attempted')
            return False

        rospy.loginfo('Executing Cartesian Plan...')
        
        # 13. send Trajectory
        action_req = ExecuteTrajectoryGoal()
        action_req.trajectory  = result.solution
        self._traj_action.send_goal(action_req)
        try:
            self._traj_action.wait_for_result()
            result = self._traj_action.get_result()
            return processResult(result)
        except:
            rospy.logerr('Failed while waiting for action result.')
            return False

    ## @brief Sets the planner_id used for all future planning requests.
    ## @param planner_id The string for the planner id, set to None to clear
    def setPlannerId(self, planner_id):
        self.planner_id = str(planner_id)

    ## @brief Set default planning time to be used for future planning request.
    def setPlanningTime(self, time):
        self.planning_time = time

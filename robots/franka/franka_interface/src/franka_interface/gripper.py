# /***************************************************************************

# 
# @package: franka_interface
# @metapackage: franka_ros_interface
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2020, Saif Sidhik
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

"""
 @info: 
   Inteface Class for Franka gripper.

"""

import rospy
import actionlib
import franka_dataflow
from copy import deepcopy
from sensor_msgs.msg import JointState

from franka_gripper.msg import ( GraspAction, GraspGoal, 
                                 HomingAction, HomingGoal,   
                                 MoveAction, MoveGoal,
                                 StopAction, StopGoal,
                                 GraspEpsilon )

class GripperInterface(object):
    """
    Interface class for the gripper on the Franka Panda robot.


    :param gripper_joint_names: Names of the finger joints
    :param ns: base namespace of interface ('frank_ros_interface'/'panda_simulator')
    :param calibrate: Attempts to calibrate the gripper when initializing class (defaults True)

    :type calibrate: bool
    :type gripper_joint_names: [str]
    :type ns: str

    """

    def __init__(self, gripper_joint_names = ('panda_finger_joint1', 'panda_finger_joint2'), calibrate = False, **kwargs):
        """
        Constructor.
        """
        
        self.name = '/franka_gripper'

        ns = self.name +'/'

        self._joint_positions = dict()
        self._joint_names = gripper_joint_names
        self._joint_velocity = dict()
        self._joint_effort = dict()

        self._joint_states_state_sub = rospy.Subscriber(ns + 'joint_states', JointState, self._joint_states_callback, queue_size = 1, tcp_nodelay = True)

        self._exists = False

        # ----- Initial test to see if gripper is loaded
        try:
            rospy.get_param("/franka_gripper/robot_ip")
        except KeyError:
            rospy.loginfo("FrankaGripper: could not detect gripper.")
            return
        except (socket.error, socket.gaierror):
            print ("Failed to connect to the ROS parameter server!\n"
           "Please check to make sure your ROS networking is "
           "properly configured:\n")
            sys.exit()

        # ----- Wait for the gripper device status to be true
        if not franka_dataflow.wait_for(lambda: len(self._joint_positions.keys()) > 0, timeout=2.0, timeout_msg=("FrankaGripper: Failed to get gripper joint positions. Assuming no gripper attached to robot."), raise_on_error = False ):
            return 
        self._exists = True

        self._gripper_speed = 0.05

        self._homing_action_client = actionlib.SimpleActionClient("{}homing".format(ns), HomingAction)

        self._grasp_action_client = actionlib.SimpleActionClient("{}grasp".format(ns), GraspAction)

        self._move_action_client = actionlib.SimpleActionClient("{}move".format(ns), MoveAction)

        self._stop_action_client = actionlib.SimpleActionClient("{}stop".format(ns), StopAction)


        rospy.loginfo("GripperInterface: Waiting for gripper action servers... ")
        self._homing_action_client.wait_for_server()
        self._grasp_action_client.wait_for_server()
        self._move_action_client.wait_for_server()
        self._stop_action_client.wait_for_server()
        rospy.loginfo("GripperInterface: Gripper action servers found! ")

        self.MIN_FORCE = 0.01
        self.MAX_FORCE = 50 # documentation says upto 70N is possible as continuous force (max upto 140N)

        self.MIN_WIDTH = 0.0001
        self.MAX_WIDTH = 0.2

        if calibrate:
            self.calibrate()



    @property
    def exists(self):
        """
        Check if a gripper was identified as connected to the robot.

        :return: True if gripper was detected, False otherwise
        :rtype: bool
        """
        return self._exists

    def set_velocity(self, value):
        """
        Set default value for gripper joint motions. Used for move and grasp commands.
       
        :param value: speed value [m/s]
        :type value: float
       
        """
        assert self.MIN_WIDTH <= value <= self.MAX_WIDTH, "GripperInterface: Invalid speed request for gripper joints. Should be within {} and {}.".format(self.MIN_WIDTH, self.MAX_WIDTH)
        self._gripper_speed = value        


    def _joint_states_callback(self, msg):

        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_positions[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.

        :rtype: [str]
        :return: ordered list of joint names.
        """
        return self._joint_names

    def joint_position(self, joint):
        """
        Return the requested joint position.

        :param joint: name of a joint
        :type joint: str

        :rtype: float
        :return: position individual joint
        """
        return self._joint_positions[joint]

    def joint_positions(self):
        """
        Return all joint positions.

        :rtype: dict({str:float})
        :return: unordered dict of joint name Keys to pos
        """
        return deepcopy(self._joint_positions)

    def joint_ordered_positions(self):
        """
        Return all joint positions.

        :rtype: [double]
        :return: joint positions ordered by joint_names.
        """
        return [self._joint_positions[name] for name in self._joint_names]

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        :param joint: name of a joint
        :type joint: str

        :rtype: float
        :return: velocity in radians/s of individual joint
        """
        return self._joint_velocity[joint]

    def joint_velocities(self):
        """
        Return all joint velocities.

        :rtype: dict({str:float})
        :return: unordered dict of joint name Keys to velocity (rad/s) Values
        """
        return deepcopy(self._joint_velocity)

    def joint_ordered_velocities(self):
        """
        Return all joint velocities.

        :rtype: [double]
        :return: joint velocities ordered by joint_names.
        """
        return [self._joint_velocity[name] for name in self._joint_names]


    def joint_effort(self, joint):
        """
        Return the requested joint effort.

        :param joint: name of a joint
        :type joint: str

        :rtype: float
        :return: effort in Nm of individual joint
        """
        return self._joint_effort[joint]

    def joint_efforts(self):
        """
        Return all joint efforts.

        :rtype: dict({str:float})
        :return: unordered dict of joint name Keys to effort (Nm) Values
        """
        return deepcopy(self._joint_effort)

    def joint_ordered_efforts(self):
        """
        Return all joint efforts.

        :rtype: [double]
        :return: joint efforts ordered by joint_names.
        """
        return [self._joint_effort[name] for name in self._joint_names]

    def _active_cb(self):
        rospy.logdebug("GripperInterface: '{}' request active.".format(self._caller))

    def _feedback_cb(self, msg):
        rospy.logdebug("GripperInterface: '{}' request feedback: \n\t{}".format(self._caller,msg))

    def _done_cb(self, status, result):
        rospy.logdebug("GripperInterface: '{}' complete. Result: \n\t{}".format(self._caller, result))


    def home_joints(self, wait_for_result = False):
        """
        Performs homing of the gripper.
       
        After changing the gripper fingers, a homing needs to be done.
        This is needed to estimate the maximum grasping width.

        :param wait_for_result: if True, this method will block till response is 
         recieved from server
        :type wait_for_result: bool
       
        :return: success
        :rtype: bool      
        
        """
        self._caller = "home_joints"

        goal = HomingGoal()

        self._homing_action_client.send_goal(goal, done_cb =self._done_cb, active_cb = self._active_cb, feedback_cb = self._feedback_cb)

        if wait_for_result:
            result = self._homing_action_client.wait_for_result(rospy.Duration(15.))
            return result

        return True

    def open(self):
        """
        Open gripper to max possible width.

        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        self._caller = "open gripper"
        return self.move_joints(0.2)

    def close(self):
        """
        close gripper to till collision is detected.
        Note: This is not exactly doing what it should. The behaviour is 
        faked by catching the error thrown when trying to grasp a very small
        object with a very small force. Since the gripper will actually hit the
        object before it reaches the commanded width, we catch the feedback 
        and send the gripper stop command to stop it where it is.

        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        def cb( _, result):
            if not result.success:
                self.stop_action()
        self._caller = "close gripper"
        return self.grasp(0.001, 0.1, cb = cb)

    def calibrate(self):
        return self.home_joints(wait_for_result = True)

    def move_joints(self, width, speed = None, wait_for_result = True):
        """
        Moves the gripper fingers to a specified width.
       
        :param width: Intended opening width. [m]
        :param speed: Closing speed. [m/s]
        :param wait_for_result: if True, this method will block till response is 
                                    recieved from server

        :type width: float
        :type speed: float
        :type wait_for_result: bool
       
        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        self._caller = "move_joints"

        goal = MoveGoal()
        if not speed:
            speed = self._gripper_speed
        goal.width = width
        goal.speed = speed

        self._move_action_client.send_goal(goal, done_cb =self._done_cb, active_cb = self._active_cb, feedback_cb = self._feedback_cb)

        if wait_for_result:
            result = self._move_action_client.wait_for_result(rospy.Duration(15.))
            return result

        return True


    def stop_action(self):
        """
        Stops a currently running gripper move or grasp.
       
        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        self._caller = "stop_action"

        goal = StopGoal()

        self._stop_action_client.send_goal(goal, done_cb =self._done_cb, active_cb = self._active_cb, feedback_cb = self._feedback_cb)

        result = self._stop_action_client.wait_for_result(rospy.Duration(15.))
        return result

    def grasp(self, width, force, speed = None, epsilon_inner = 0.005, epsilon_outer = 0.005,wait_for_result = True, cb = None):
        """
        Grasps an object.
       
        An object is considered grasped if the distance $d$ between the gripper fingers satisfies
        $(\text{width} - \text{epsilon_inner}) < d < (\text{width} + \text{epsilon_outer})$.
       
        :param width: Size of the object to grasp. [m]
        :param speed: Closing speed. [m/s]
        :param force: Grasping force. [N]
        :param epsilon_inner: Maximum tolerated deviation when the actual grasped width is smaller
                                than the commanded grasp width.
        :param epsilon_outer: Maximum tolerated deviation when the actual grasped width is wider
                                than the commanded grasp width.
        :param cb: Optional callback function to use when the service call is done

        :type width: float
        :type speed: float
        :type force: float
        :type epsilon_inner: float
        :type epsilon_outer: float

        :return: True if an object has been grasped, false otherwise.
        :rtype: bool
        """
        self._caller = "grasp_action"

        if not speed:
            speed = self._gripper_speed

        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner = epsilon_inner, outer = epsilon_outer)

        if not cb:
            cb = self._done_cb

        self._grasp_action_client.send_goal(goal, done_cb = cb, active_cb = self._active_cb, feedback_cb = self._feedback_cb)

        if wait_for_result:
            result = self._grasp_action_client.wait_for_result(rospy.Duration(15.))
            return result

        return True


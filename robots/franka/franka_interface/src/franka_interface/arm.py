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
   Inteface Class for Franka robot arm.

"""


import enum
import rospy
import warnings
import quaternion
import numpy as np
from copy import deepcopy
from rospy_message_converter import message_converter

from franka_core_msgs.msg import JointCommand
from franka_core_msgs.msg import RobotState, EndPointState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import franka_dataflow
from robot_params import RobotParams

from franka_moveit import PandaMoveGroupInterface
from franka_tools import FrankaFramesInterface, FrankaControllerManagerInterface, JointTrajectoryActionClient, CollisionBehaviourInterface


class TipState():

    def __init__(self, timestamp, pose, vel, O_effort, K_effort):
        self.timestamp = timestamp
        self._pose = pose
        self._velocity = vel
        self._effort = O_effort
        self._effort_in_K_frame = K_effort

    @property
    def pose(self):
        return self._pose
    @property
    def velocity(self):
        return self._velocity
    @property
    def effort(self):
        return self._effort
    @property
    def effort_in_K_frame(self):
        return self._effort_in_K_frame

    
    
    

class ArmInterface(object):

    """ 
    Interface Class for an arm of Franka Panda robot
    Constructor.
    
    :type synchronous_pub: bool
    :param synchronous_pub: designates the JointCommand Publisher
        as Synchronous if True and Asynchronous if False.

        Synchronous Publishing means that all joint_commands publishing to
        the robot's joints will block until the message has been serialized
        into a buffer and that buffer has been written to the transport
        of every current Subscriber. This yields predicable and consistent
        timing of messages being delivered from this Publisher. However,
        when using this mode, it is possible for a blocking Subscriber to
        prevent the joint_command functions from exiting. Unless you need exact
        JointCommand timing, default to Asynchronous Publishing (False).
    """

    # Containers
    @enum.unique
    class RobotMode(enum.IntEnum):
        """
            Enum class for specifying and retrieving the current robot mode.
        """
        # ----- access using parameters name or value
        # ----- eg. RobotMode(0).name & RobotMode(0).value
        # ----- or  RobotMode['ROBOT_MODE_OTHER'].name & RobotMode['ROBOT_MODE_OTHER'].value

        ROBOT_MODE_OTHER                        = 0
        ROBOT_MODE_IDLE                         = 1
        ROBOT_MODE_MOVE                         = 2
        ROBOT_MODE_GUIDING                      = 3
        ROBOT_MODE_REFLEX                       = 4
        ROBOT_MODE_USER_STOPPED                 = 5
        ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY     = 6

    def __init__(self, synchronous_pub=False):
        """

        """

        self._params = RobotParams()

        self._ns = self._params.get_base_namespace()

        self._joint_limits = self._params.get_joint_limits()

        joint_names = self._joint_limits.joint_names
        if not joint_names:
            rospy.logerr("Cannot detect joint names for arm on this "
                         "robot. Exiting Arm.init().")
                         
            return   

        self._joint_names = joint_names
        self.name = self._params.get_robot_name()
        self._joint_angle = dict()
        self._joint_velocity = dict()
        self._joint_effort = dict()
        self._cartesian_pose = dict()
        self._cartesian_velocity = dict()
        self._cartesian_effort = dict()
        self._stiffness_frame_effort = dict()
        self._errors = dict()
        self._collision_state = False
        self._tip_states = None
        self._jacobian = None
        self._cartesian_contact = None

        self._robot_mode = False

        self._command_msg = JointCommand()

        # neutral pose joint positions
        self._neutral_pose_joints = self._params.get_neutral_pose()

        self._frames_interface = FrankaFramesInterface()
        try:
            self._collision_behaviour_interface = CollisionBehaviourInterface()
        except rospy.ROSException:
            rospy.loginfo("Collision Service Not found. It will not be possible to change collision behaviour of robot!")
            self._collision_behaviour_interface = None
        self._ctrl_manager = FrankaControllerManagerInterface(ns = self._ns, sim = self._params._in_sim)

        self._speed_ratio = 0.15

        queue_size = None if synchronous_pub else 1
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._joint_command_publisher = rospy.Publisher(
                self._ns +'/motion_controller/arm/joint_commands',
                JointCommand,
                tcp_nodelay=True,
                queue_size=queue_size)

        self._pub_joint_cmd_timeout = rospy.Publisher(
            self._ns +'/motion_controller/arm/joint_command_timeout',
            Float64,
            latch=True,
            queue_size=10)



        self._robot_state_subscriber = rospy.Subscriber(
            self._ns + '/custom_franka_state_controller/robot_state',
            RobotState,
            self._on_robot_state,
            queue_size=1,
            tcp_nodelay=True)

        joint_state_topic = self._ns + '/custom_franka_state_controller/joint_states'
        self._joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

        self._cartesian_state_sub = rospy.Subscriber(
            self._ns + '/custom_franka_state_controller/tip_state',
            EndPointState,
            self._on_endpoint_state,
            queue_size=1,
            tcp_nodelay=True)

        rospy.on_shutdown(self._clean_shutdown)

        err_msg = ("%s arm init failed to get current joint_states "
                   "from %s") % (self.name.capitalize(), joint_state_topic)
        franka_dataflow.wait_for(lambda: len(self._joint_angle.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)

        err_msg = ("%s arm, init failed to get current tip_state "
                   "from %s") % (self.name.capitalize(), self._ns + 'tip_state')
        franka_dataflow.wait_for(lambda: len(self._cartesian_pose.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)

        err_msg = ("%s arm, init failed to get current robot_state "
                   "from %s") % (self.name.capitalize(), self._ns + 'robot_state')
        franka_dataflow.wait_for(lambda: self._jacobian is not None,
                                 timeout_msg=err_msg, timeout=5.0)

        try:
            self._movegroup_interface = PandaMoveGroupInterface()
        except:
            rospy.loginfo("MoveGroup was not found! This is okay if moveit service is not required!")
            self._movegroup_interface = None

        self.set_joint_position_speed(self._speed_ratio)

    def _clean_shutdown(self):
        self._joint_state_sub.unregister()
        self._cartesian_state_sub.unregister()
        self._pub_joint_cmd_timeout.unregister()
        self._robot_state_subscriber.unregister()
        self._joint_command_publisher.unregister()

    def get_movegroup_interface(self):
        """
        :return: the movegroup interface instance associated with the robot.
        :rtype: franka_moveit.PandaMoveGroupInterface
        """
        return self._movegroup_interface

    def get_robot_params(self):
        """
        :return: Useful parameters from the ROS parameter server.
        :rtype: franka_interface.RobotParams
        """
        return self._params

    def get_joint_limits(self):
        """
        Return the joint limits (defined in the parameter server)

        :rtype: franka_core_msgs.msg.JointLimits
        :return: JointLimits
        """
        return self._joint_limits

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.

        :rtype: [str]
        :return: ordered list of joint names from proximal to distal (i.e. shoulder to wrist).
        """
        return self._joint_names

    def _on_joint_states(self, msg):

        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_angle[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    def _on_robot_state(self, msg):

        self._robot_mode = self.RobotMode(msg.robot_mode)

        self._robot_mode_ok = (self._robot_mode.value != self.RobotMode.ROBOT_MODE_REFLEX) and (self._robot_mode.value != self.RobotMode.ROBOT_MODE_USER_STOPPED)

        self._jacobian = np.asarray(msg.O_Jac_EE).reshape(6,7,order = 'F')

        self._cartesian_velocity = {
                'linear': np.asarray([msg.O_dP_EE[0], msg.O_dP_EE[1], msg.O_dP_EE[2]]),
                'angular': np.asarray([msg.O_dP_EE[3], msg.O_dP_EE[4], msg.O_dP_EE[5]]) }

        self._cartesian_contact = msg.cartesian_contact
        self._cartesian_collision = msg.cartesian_collision

        self._joint_contact = msg.joint_contact
        self._joint_collision = msg.joint_collision
        if self._frames_interface:
            self._frames_interface._update_frame_data(msg.F_T_EE, msg.EE_T_K)

        self._joint_inertia = np.asarray(msg.mass_matrix).reshape(7,7,order='F')

        self.q_d = msg.q_d
        self.dq_d = msg.dq_d

        self._gravity = np.asarray(msg.gravity)
        self._coriolis = np.asarray(msg.coriolis)

        self._errors = message_converter.convert_ros_message_to_dictionary(msg.current_errors)

    def coriolis_comp(self):
        """
        Return coriolis compensation torques. Useful for compensating coriolis when
        performing direct torque control of the robot.

        :rtype: np.ndarray
        :return: 7D joint torques compensating for coriolis.
        """
        return self._coriolis
        
    def gravity_comp(self):
        """
        Return gravity compensation torques.

        :rtype: np.ndarray
        :return: 7D joint torques compensating for gravity.
        """
        return self._gravity

    def get_robot_status(self):
        """
        Return dict with all robot status information.

        :rtype: dict
        :return: ['robot_mode' (RobotMode object), 'robot_status' (bool), 'errors' (dict() of errors and their truth value), 'error_in_curr_status' (bool)]
        """
        return {'robot_mode': self._robot_mode, 'robot_status': self._robot_mode_ok, 'errors': self._errors, 'error_in_current_state' : self.error_in_current_state()}

    def in_safe_state(self):
        """
        Return True if the specified limb is in safe state (no collision, reflex, errors etc.).

        :rtype: bool
        :return: True if the arm is in safe state, False otherwise.
        """
        return self._robot_mode_ok and not self.error_in_current_state()

    def error_in_current_state(self):
        """
        Return True if the specified limb has experienced an error.

        :rtype: bool
        :return: True if the arm has error, False otherwise.
        """
        return not all([e == False for e in self._errors.values()])

    def what_errors(self):
        """
        Return list of error messages if there is error in robot state

        :rtype: [str]
        :return: list of names of current errors in robot state
        """
        return [e for e in self._errors if self._errors[e] == True] if self.error_in_current_state() else None


    def _on_endpoint_state(self, msg):

        cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4,4,order='F')

        self._cartesian_pose = {
            'position': cart_pose_trans_mat[:3,3],
            'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }

        self._cartesian_effort = {
            'force': np.asarray([ msg.O_F_ext_hat_K.wrench.force.x,
                                  msg.O_F_ext_hat_K.wrench.force.y,
                                  msg.O_F_ext_hat_K.wrench.force.z]),

            'torque': np.asarray([ msg.O_F_ext_hat_K.wrench.torque.x,
                                   msg.O_F_ext_hat_K.wrench.torque.y,
                                   msg.O_F_ext_hat_K.wrench.torque.z])
        }

        self._stiffness_frame_effort = {
            'force': np.asarray([ msg.K_F_ext_hat_K.wrench.force.x,
                                  msg.K_F_ext_hat_K.wrench.force.y,
                                  msg.K_F_ext_hat_K.wrench.force.z]),

            'torque': np.asarray([ msg.K_F_ext_hat_K.wrench.torque.x,
                                   msg.K_F_ext_hat_K.wrench.torque.y,
                                   msg.K_F_ext_hat_K.wrench.torque.z])
        }

        self._tip_states = TipState(msg.header.stamp, deepcopy(self._cartesian_pose), deepcopy(self._cartesian_velocity), deepcopy(self._cartesian_effort), deepcopy(self._stiffness_frame_effort))



    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        :type joint: str
        :param joint: name of a joint
        :rtype: float
        :return: angle in radians of individual joint
        """
        return self._joint_angle[joint]

    def joint_angles(self):
        """
        Return all joint angles.

        :rtype: dict({str:float})
        :return: unordered dict of joint name Keys to angle (rad) Values
        """
        return deepcopy(self._joint_angle)

    def joint_ordered_angles(self):
        """
        Return all joint angles.

        :rtype: [float]
        :return: joint angles (rad) orded by joint_names from proximal to distal (i.e. shoulder to wrist).
        """
        return [self._joint_angle[name] for name in self._joint_names]

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        :type joint: str
        :param joint: name of a joint
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

    def joint_effort(self, joint):
        """
        Return the requested joint effort.

        :type joint: str
        :param joint: name of a joint
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

    def endpoint_pose(self):
        """
        Return Cartesian endpoint pose {position, orientation}.

        :rtype: dict({str:np.ndarray (shape:(3,)), str:quaternion.quaternion})
        :return: position and orientation as named tuples in a dict

          - 'position': np.array of x, y, z
          - 'orientation': quaternion x,y,z,w in quaternion format

        """
        return deepcopy(self._cartesian_pose)

    def endpoint_velocity(self):
        """
        Return Cartesian endpoint twist {linear, angular}.

        :rtype: dict({str:np.ndarray (shape:(3,)),str:np.ndarray (shape:(3,))})
        :return: linear and angular velocities as named tuples in a dict

          - 'linear': np.array of x, y, z
          - 'angular': np.array of x, y, z (angular velocity along the axes)
        """
        return deepcopy(self._cartesian_velocity)

    def endpoint_effort(self):
        """
        Return Cartesian endpoint wrench {force, torque}.

        :rtype: dict({str:np.ndarray (shape:(3,)),str:np.ndarray (shape:(3,))})
        :return: force and torque at endpoint as named tuples in a dict

          - 'force': Cartesian force on x,y,z axes in np.ndarray format
          - 'torque': Torque around x,y,z axes in np.ndarray format
        """
        return deepcopy(self._cartesian_effort)

    def exit_control_mode(self, timeout=0.2):
        """
        Clean exit from advanced control modes (joint torque or velocity).
        Resets control to joint position mode with current positions if the 
        advanced control commands are not send within the specified timeout
        interval.

        :type timeout: float
        :param timeout: control timeout in seconds [default: 0.2]
        """
        self.set_command_timeout(timeout)
        self.set_joint_positions(self.joint_angles())

    def tip_states(self):
        """
        Return Cartesian endpoint state for a given tip name

        :rtype: TipState object
        :return: pose, velocity, effort, effort_in_K_frame
        """
        return deepcopy(self._tip_states)
        
    def joint_inertia_matrix(self):
        """
        
        :return: joint inertia matrix (7,7)
        :rtype: np.ndarray [7x7]
        """
        return deepcopy(self._joint_inertia)

    def zero_jacobian(self):
        """
        :return: end-effector jacobian (6,7)
        :rtype: np.ndarray [6x7]
        """
        return deepcopy(self._jacobian)        

    def set_command_timeout(self, timeout):
        """
        Set the timeout in seconds for the joint controller

        :type timeout: float
        :param timeout: timeout in seconds
        """
        self._pub_joint_cmd_timeout.publish(Float64(timeout))


    def set_joint_position_speed(self, speed=0.3):
        """
        Set ratio of max joint speed to use during joint position 
        moves (only for move_to_joint_positions).

        Set the proportion of maximum controllable velocity to use
        during joint position control execution. The default ratio
        is `0.3`, and can be set anywhere from [0.0-1.0] (clipped).
        Once set, a speed ratio will persist until a new execution
        speed is set.

        :type speed: float
        :param speed: ratio of maximum joint speed for execution
                      default= 0.3; range= [0.0-1.0]
        """
        if speed > 0.3:
            rospy.logwarn("ArmInterface: Setting speed above 0.3 could be risky!! Be extremely careful.")
        if self._movegroup_interface:
            self._movegroup_interface.set_velocity_scale(speed * 2)
        self._speed_ratio = speed

    def set_joint_positions(self, positions):
        """
        Commands the joints of this limb to the specified positions.

        :type positions: dict({str:float}
        :param positions: dict of {'joint_name':joint_position,}
        """
        self._command_msg.names = self._joint_names
        self._command_msg.position = [positions[j] for j in self._joint_names]
        self._command_msg.mode = JointCommand.POSITION_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def set_joint_velocities(self, velocities):
        """
        Commands the joints of this limb to the specified velocities.

        :type velocities: dict({str:float})
        :param velocities: dict of {'joint_name':joint_velocity,}
        """
        self._command_msg.names = self._joint_names
        self._command_msg.velocity = [velocities[j] for j in self._joint_names]
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def set_joint_torques(self, torques):
        """
        Commands the joints of this limb with the specified torques.

        :type torques: dict({str:float})
        :param torques: dict of {'joint_name':joint_torque,}
        """
        self._command_msg.names = self._joint_names
        self._command_msg.effort = [torques[j] for j in self._joint_names]
        self._command_msg.mode = JointCommand.TORQUE_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def set_joint_positions_velocities(self, positions, velocities):
        """
        Commands the joints of this limb using specified positions and velocities using impedance control.
        Command at time t is computed as:

        :math:`u_t= coriolis\_factor * coriolis\_t + K\_p * (positions - curr\_positions) +  K\_d * (velocities - curr\_velocities)`


        :type positions: [float]
        :param positions: desired joint positions as an ordered list corresponding to joints given by self.joint_names()
        :type velocities: [float]
        :param velocities: desired joint velocities as an ordered list corresponding to joints given by self.joint_names()
        """
        self._command_msg.names = self._joint_names
        self._command_msg.position = positions
        self._command_msg.velocity = velocities
        self._command_msg.mode = JointCommand.IMPEDANCE_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)


    def has_collided(self):
        """
        Returns true if either joint collision or cartesian collision is detected. 
        Collision thresholds can be set using instance of :py:class:`franka_tools.CollisionBehaviourInterface`.
        """
        return any(self._joint_collision) or any(self._cartesian_collision)
        

    def move_to_neutral(self, timeout=15.0, speed=0.15):
        """
        Command the Limb joints to a predefined set of "neutral" joint angles.
        From rosparam /franka_control/neutral_pose.

        :type timeout: float
        :param timeout: seconds to wait for move to finish [15]
        :type speed: float
        :param speed: ratio of maximum joint speed for execution
         default= 0.15; range= [0.0-1.0]
        """
        self.set_joint_position_speed(speed)
        self.move_to_joint_positions(self._neutral_pose_joints, timeout)


    def move_to_joint_positions(self, positions, timeout=10.0,
                                threshold=0.00085,
                                test=None, use_moveit = True):
        """
        (Blocking) Commands the limb to the provided positions.
        Waits until the reported joint state matches that specified.
        This function uses a low-pass filter using JointTrajectoryService 
        to smooth the movement or optionally uses MoveIt! to plan and 
        execute a trajectory.

        :type positions: dict({str:float})
        :param positions: joint_name:angle command
        :type timeout: float
        :param timeout: seconds to wait for move to finish [15]
        :type threshold: float
        :param threshold: position threshold in radians across each joint when
         move is considered successful [0.00085]
        :param test: optional function returning True if motion must be aborted
        :type use_moveit: bool
        :param use_moveit: if set to True, and movegroup interface is available, 
         move to the joint positions using moveit planner.
        """

        curr_controller = self._ctrl_manager.set_motion_controller(self._ctrl_manager.joint_trajectory_controller)

        if use_moveit and self._movegroup_interface:
            self._movegroup_interface.go_to_joint_positions([positions[n] for n in self._joint_names], tolerance = threshold)
        
        else:
            if use_moveit:
                rospy.logwarn("ArmInterface: MoveGroupInterface was not found! Using JointTrajectoryActionClient instead.")
            min_traj_dur = 0.5
            traj_client = JointTrajectoryActionClient(joint_names = self.joint_names())
            traj_client.clear()

            dur = []
            for j in range(len(self._joint_names)):
                dur.append(max(abs(positions[self._joint_names[j]] - self._joint_angle[self._joint_names[j]]) / self._joint_limits.velocity[j], min_traj_dur))

            traj_client.add_point(positions = [self._joint_angle[n] for n in self._joint_names], time = 0.0001)
            traj_client.add_point(positions = [positions[n] for n in self._joint_names], time = max(dur)/self._speed_ratio)

            def genf(joint, angle):
                def joint_diff():
                    return abs(angle - self._joint_angle[joint])
                return joint_diff

            diffs = [genf(j, a) for j, a in positions.items() if
                     j in self._joint_angle]

            fail_msg = "ArmInterface: {0} limb failed to reach commanded joint positions.".format(
                                                          self.name.capitalize())
            def test_collision():
                if self.has_collided():
                    rospy.logerr(' '.join(["Collision detected.", fail_msg]))
                    return True
                return False

            traj_client.start() # send the trajectory action request
            # traj_client.wait(timeout = timeout)

            franka_dataflow.wait_for(
                test=lambda: test_collision() or traj_client.result() is not None or \
                             (callable(test) and test() == True) or \
                             (all(diff() < threshold for diff in diffs)),
                timeout=timeout,
                timeout_msg=fail_msg,
                rate=100,
                raise_on_error=False
                )
            res = traj_client.result()
            if res is not None and res.error_code:
                rospy.loginfo("Trajectory Server Message: {}".format(res))

        rospy.sleep(0.5)

        self._ctrl_manager.set_motion_controller(curr_controller)

        rospy.loginfo("ArmInterface: Trajectory controlling complete")

    def pause_controllers_and_do(self, func, *args, **kwargs):
        """
        Temporarily stops all active controllers and calls the provided function
        before restarting the previously active controllers.

        :param func: the function to be called
        :type func: callable
        """
        assert callable(func), "ArmInterface: Invalid argument provided to ArmInterface->pause_controllers_and_do. Argument 1 should be callable."
        active_controllers = self._ctrl_manager.list_active_controllers(only_motion_controllers = True)

        rospy.loginfo("ArmInterface: Stopping motion controllers temporarily...")
        for ctrlr in active_controllers:
            self._ctrl_manager.stop_controller(ctrlr.name)
        rospy.sleep(1.)

        retval = func(*args, **kwargs)

        rospy.sleep(1.)
        rospy.loginfo("ArmInterface: Restarting previously active motion controllers.")
        for ctrlr in active_controllers:
            self._ctrl_manager.start_controller(ctrlr.name)
        rospy.sleep(1.)
        rospy.loginfo("ArmInterface: Controllers restarted.")

        return retval


    def reset_EE_frame(self):
        """
        Reset EE frame to default. (defined by 
        FrankaFramesInterface.DEFAULT_TRANSFORMATIONS.EE_FRAME 
        global variable defined in :py:class:`franka_tools.FrankaFramesInterface` 
        source code) 

        :rtype: [bool, str]
        :return: [success status of service request, error msg if any]
        """
        if self._frames_interface:

            if self._frames_interface.EE_frame_is_reset():
                rospy.loginfo("ArmInterface: EE Frame already reset")
                return

            return self.pause_controllers_and_do(self._frames_interface.reset_EE_frame)

        else:
            rospy.logwarn("ArmInterface: Frames changing not available in simulated environment")
            return False


    def set_EE_frame(self, frame):
        """
        Set new EE frame based on the transformation given by 'frame', which is the 
        transformation matrix defining the new desired EE frame with respect to the flange frame.
        Motion controllers are stopped for switching

        :type frame: [float (16,)] / np.ndarray (4x4) 
        :param frame: transformation matrix of new EE frame wrt flange frame (column major)
        :rtype: [bool, str]
        :return: [success status of service request, error msg if any]
        """
        if self._frames_interface:

            if self._frames_interface.frames_are_same(self._frames_interface.get_EE_frame(as_mat=True), frame):
                rospy.loginfo("ArmInterface: EE Frame already at the target frame.")
                return True

            return self.pause_controllers_and_do(self._frames_interface.set_EE_frame,frame)

        else:
            rospy.logwarn("ArmInterface: Frames changing not available in simulated environment")

    def set_EE_frame_to_link(self, frame_name, timeout = 5.0):
        """
        Set new EE frame to the same frame as the link frame given by 'frame_name'
        Motion controllers are stopped for switching

        :type frame_name: str 
        :param frame_name: desired tf frame name in the tf tree
        :rtype: [bool, str]
        :return: [success status of service request, error msg if any]
        """
        if self._frames_interface:
            retval = True
            if not self._frames_interface.EE_frame_already_set(self._frames_interface.get_link_tf(frame_name)):

                return self.pause_controllers_and_do(self._frames_interface.set_EE_frame_to_link,frame_name = frame_name, timeout = timeout)

        else:
            rospy.logwarn("ArmInterface: Frames changing not available in simulated environment")

    def set_collision_threshold(self, cartesian_forces = None, joint_torques = None):
        """
        Set Force Torque thresholds for deciding robot has collided.

        :return: True if service call successful, False otherwise
        :rtype: bool
        :param cartesian_forces: Cartesian force threshold for collision detection [x,y,z,R,P,Y] (robot motion stops if violated)
        :type cartesian_forces: [float] size 6
        :param joint_torques: Joint torque threshold for collision (robot motion stops if violated)
        :type joint_torques: [float] size 7
        """
        if self._collision_behaviour_interface:
            return self._collision_behaviour_interface.set_collision_threshold(joint_torques = joint_torques, cartesian_forces = cartesian_forces)
        else:
            rospy.logwarn("No CollisionBehaviourInterface object found!")

    def get_controller_manager(self):
        """
        :return: the FrankaControllerManagerInterface instance associated with the robot.
        :rtype: franka_tools.FrankaControllerManagerInterface
        """
        return self._ctrl_manager


    def get_frames_interface(self):
        """
        :return: the FrankaFramesInterface instance associated with the robot.
        :rtype: franka_tools.FrankaFramesInterface
        """
        return self._frames_interface


if __name__ == '__main__':
    rospy.init_node('test')
    r = Arm()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import copy
import importlib
import os
import sys
import threading
import time
from abc import ABCMeta, abstractmethod

import PyKDL as kdl
import moveit_commander
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from kdl_parser_py.urdf import treeFromParam
from sensor_msgs.msg import JointState
from trac_ik_python import trac_ik

import util as prutil


class Robot:
    """
    This is the main interface class that is composed of
    key robot modules (base, arm, gripper, and camera).
    This class builds robot specific objects by reading a
    configuration and instantiating the necessary robot module objects.

    """

    def __init__(self,
                 robot_name,
                 use_arm=True,
                 use_base=True,
                 use_camera=True,
                 use_gripper=True,
                 arm_config={},
                 base_config={},
                 camera_config={},
                 gripper_config={}):
        """
        Constructor for the Robot class

        :param robot_name: robot name
        :param use_arm: use arm or not
        :param use_base: use base or not
        :param use_camera: use camera or not
        :param use_gripper: use gripper or not
        :param arm_config: configurations for arm
        :param base_config: configurations for base
        :param camera_config: configurations for camera
        :param gripper_config: configurations for gripper

        :type robot_name: string
        :type use_arm: bool
        :type use_base: bool
        :type use_camera: bool
        :type use_gripper: bool
        :type arm_config: dict
        :type base_config: dict
        :type camera_config: dict
        :type gripper_config: dict
        """
        root_path = os.path.dirname(os.path.realpath(__file__))
        cfg_path = os.path.join(root_path, 'cfg')
        robot_pool = []
        for f in os.listdir(cfg_path):
            if f.endswith('_config.py'):
                robot_pool.append(f[:-len('_config.py')])
        root_node = 'pyrobot.'
        self.configs = None
        this_robot = None
        for srobot in robot_pool:
            if srobot in robot_name:
                this_robot = srobot
                mod = importlib.import_module(root_node + 'cfg.' +
                                              '{:s}_config'.format(srobot))
                cfg_func = getattr(mod, 'get_cfg')
                if srobot == 'locobot' and 'lite' in robot_name:
                    self.configs = cfg_func('create')
                else:
                    self.configs = cfg_func()
        if self.configs is None:
            raise ValueError('Invalid robot name provided, only the following'
                             ' are currently available: {}'.format(robot_pool))
        self.configs.freeze()
        try:
            rospy.init_node('pyrobot', anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn('ROS node [pyrobot] has already been initialized')

        root_node += this_robot
        root_node += '.'
        if self.configs.HAS_ARM and use_arm:
            mod = importlib.import_module(root_node + 'arm')
            arm_class = getattr(mod, self.configs.ARM.CLASS)
            self.arm = arm_class(self.configs, **arm_config)
        if self.configs.HAS_BASE and use_base:
            mod = importlib.import_module(root_node + 'base')
            base_class = getattr(mod, self.configs.BASE.CLASS)
            self.base = base_class(self.configs, **base_config)
        if self.configs.HAS_CAMERA and use_camera:
            mod = importlib.import_module(root_node + 'camera')
            camera_class = getattr(mod, self.configs.CAMERA.CLASS)
            self.camera = camera_class(self.configs, **camera_config)
        if self.configs.HAS_GRIPPER and use_gripper and use_arm:
            mod = importlib.import_module(root_node + 'gripper')
            gripper_class = getattr(mod, self.configs.GRIPPER.CLASS)
            self.gripper = gripper_class(self.configs, **gripper_config)

        # sleep some time for tf listeners in subclasses
        rospy.sleep(2)


class Base(object):
    """
    This is a parent class on which the robot
    specific Base classes would be built.
    """

    def __init__(self, configs):
        """
        The consturctor for Base class.

        :param configs: configurations for base
        :type configs: YACS CfgNode
        """
        self.configs = configs
        self.ctrl_pub = rospy.Publisher(configs.BASE.ROSTOPIC_BASE_COMMAND,
                                        Twist, queue_size=1)

    def stop(self):
        """
        Stop the base
        """
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.ctrl_pub.publish(msg)

    def set_vel(self, fwd_speed, turn_speed, exe_time=1):
        """
        Set the moving velocity of the base

        :param fwd_speed: forward speed
        :param turn_speed: turning speed
        :param exe_time: execution time
        """
        fwd_speed = min(fwd_speed, self.configs.BASE.MAX_ABS_FWD_SPEED)
        fwd_speed = max(fwd_speed, -self.configs.BASE.MAX_ABS_FWD_SPEED)
        turn_speed = min(turn_speed, self.configs.BASE.MAX_ABS_TURN_SPEED)
        turn_speed = max(turn_speed, -self.configs.BASE.MAX_ABS_TURN_SPEED)

        msg = Twist()
        msg.linear.x = fwd_speed
        msg.angular.z = turn_speed

        start_time = rospy.get_time()
        self.ctrl_pub.publish(msg)
        while rospy.get_time() - start_time < exe_time:
            self.ctrl_pub.publish(msg)
            rospy.sleep(1. / self.configs.BASE.BASE_CONTROL_RATE)

    def go_to_relative(self, xyt_position, use_map, close_loop, smooth):
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
        """
        raise NotImplementedError

    def go_to_absolute(self, xyt_position, use_map, close_loop, smooth):
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
        """
        raise NotImplementedError

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
        """
        raise NotImplementedError

    def get_state(self, state_type):
        """
        Returns the requested base pose in the (x,y, yaw) format.

        :param state_type: Requested state type. Ex: Odom, SLAM, etc
        :type state_type: string
        :return: pose: pose of the form [x, y, yaw]
        :rtype: list
        """
        raise NotImplementedError


class Gripper(object):
    """
    This is a parent class on which the robot
    specific Gripper classes would be built.
    """
    __metaclass__ = ABCMeta

    def __init__(self, configs):
        """
        Constructor for Gripper parent class.

        :param configs: configurations for gripper
        :type configs: YACS CfgNode
        """
        self.configs = configs

    @abstractmethod
    def open(self, **kwargs):
        pass

    @abstractmethod
    def close(self, **kwargs):
        pass


class Camera(object):
    """
    This is a parent class on which the robot
    specific Camera classes would be built.
    """
    __metaclass__ = ABCMeta

    def __init__(self, configs):
        """
        Constructor for Camera parent class.

        :param configs: configurations for camera
        :type configs: YACS CfgNode
        """
        self.configs = configs

    @abstractmethod
    def get_rgb(self, **kwargs):
        pass


class Arm(object):
    """
    This is a parent class on which the robot
    specific Arm classes would be built.
    """
    __metaclass__ = ABCMeta

    def __init__(self,
                 configs,
                 moveit_planner='ESTkConfigDefault',
                 analytical_ik=None,
                 use_moveit=True):
        """
        Constructor for Arm parent class.

        :param configs: configurations for arm
        :param moveit_planner: moveit planner type
        :param analytical_ik: customized analytical ik class
                              if you have one, None otherwise
        :param use_moveit: use moveit or not, default is True

        :type configs: YACS CfgNode
        :type moveit_planner: string
        :type analytical_ik: None or an Analytical ik class
        :type use_moveit: bool
        """
        self.configs = configs
        self.moveit_planner = moveit_planner
        self.moveit_group = None
        self.scene = None
        self.use_moveit = use_moveit

        self.joint_state_lock = threading.RLock()
        self.tf_listener = tf.TransformListener()
        if self.use_moveit:
            self._init_moveit()
        robot_description = self.configs.ARM.ARM_ROBOT_DSP_PARAM_NAME
        urdf_string = rospy.get_param(robot_description)
        self.num_ik_solver = trac_ik.IK(configs.ARM.ARM_BASE_FRAME,
                                        configs.ARM.EE_FRAME,
                                        urdf_string=urdf_string)
        if analytical_ik is not None:
            self.ana_ik_solver = analytical_ik(configs.ARM.ARM_BASE_FRAME,
                                               configs.ARM.EE_FRAME)

        _, self.urdf_tree = treeFromParam(robot_description)
        self.urdf_chain = self.urdf_tree.getChain(configs.ARM.ARM_BASE_FRAME,
                                                  configs.ARM.EE_FRAME)
        self.arm_joint_names = self._get_kdl_joint_names()
        self.arm_link_names = self._get_kdl_link_names()
        self.arm_dof = len(self.arm_joint_names)

        self.jac_solver = kdl.ChainJntToJacSolver(self.urdf_chain)
        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)
        self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)

        # Subscribers
        self._joint_angles = dict()
        self._joint_velocities = dict()
        self._joint_efforts = dict()
        rospy.Subscriber(configs.ARM.ROSTOPIC_JOINT_STATES, JointState,
                         self._callback_joint_states)

        # Publishers
        self.joint_pub = None
        self._setup_joint_pub()

    @abstractmethod
    def go_home(self):
        """
        Reset robot to default home position
        """
        pass

    @property
    def pose_ee(self):
        """
        Return the end effector pose w.r.t 'ARM_BASE_FRAME'

        :return:
                trans: translational vector (shape: :math:`[3, 1]`)

                rot_mat: rotational matrix (shape: :math:`[3, 3]`)

                quat: rotational matrix in the form
                of quaternion (shape: :math:`[4,]`)

        :rtype: tuple (trans, rot_mat, quat)
        """
        return self.get_ee_pose(base_frame=self.configs.ARM.ARM_BASE_FRAME)

    def get_ee_pose(self, base_frame):
        """
        Return the end effector pose with respect to the base_frame

        :param base_frame: reference frame
        :type base_frame: string
        :return:
                tuple (trans, rot_mat, quat)

                trans: translational vector (shape: :math:`[3, 1]`)

                rot_mat: rotational matrix (shape: :math:`[3, 3]`)

                quat: rotational matrix in the form
                of quaternion (shape: :math:`[4,]`)

        :rtype: tuple or ROS PoseStamped
        """
        return self.get_transform(base_frame, self.configs.ARM.EE_FRAME)

    def get_transform(self, src_frame, dest_frame):
        """
        Return the transform from the src_frame to dest_frame

        :param src_frame: source frame
        :param dest_frame: destination frame
        :type src_frame: string
        :type dest_frame: basestring
        :return:
               tuple (trans, rot_mat, quat )

               trans: translational vector (shape: :math:`[3, 1]`)

               rot_mat: rotational matrix (shape: :math:`[3, 3]`)

               quat: rotational matrix in the form
               of quaternion (shape: :math:`[4,]`)

        :rtype: tuple or ROS PoseStamped
        """
        trans, quat = prutil.get_tf_transform(self.tf_listener,
                                              src_frame,
                                              dest_frame)
        rot_mat = prutil.quat_to_rot_mat(quat)
        trans = np.array(trans).reshape(-1, 1)
        rot_mat = np.array(rot_mat)
        quat = np.array(quat)
        return trans, rot_mat, quat

    def get_joint_angles(self):
        """
        Return the joint angles

        :return: joint_angles
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        joint_angles = []
        for joint in self.arm_joint_names:
            joint_angles.append(self.get_joint_angle(joint))
        joint_angles = np.array(joint_angles).flatten()
        self.joint_state_lock.release()
        return joint_angles

    def get_joint_velocities(self):
        """
        Return the joint velocities

        :return: joint_vels
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        joint_vels = []
        for joint in self.arm_joint_names:
            joint_vels.append(self.get_joint_velocity(joint))
        joint_vels = np.array(joint_vels).flatten()
        self.joint_state_lock.release()
        return joint_vels

    def get_joint_torques(self):
        """
        Return the joint torques

        :return: joint_torques
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        joint_torques = []
        for joint in self.arm_joint_names:
            try:
                joint_torques.append(self.get_joint_torque(joint))
            except (ValueError, IndexError):
                rospy.loginfo('Torque value for joint '
                              '[%s] not available!' % joint)
        joint_torques = np.array(joint_torques).flatten()
        self.joint_state_lock.release()
        return joint_torques

    def get_joint_angle(self, joint):
        """
        Return the joint angle of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint angle
        :rtype: float
        """
        if joint not in self.arm_joint_names:
            raise ValueError('%s not in arm joint list!' % joint)
        if joint not in self._joint_angles.keys():
            raise ValueError('Joint angle for joint $s not available!' % joint)
        return self._joint_angles[joint]

    def get_joint_velocity(self, joint):
        """
        Return the joint velocity of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint velocity
        :rtype: float
        """
        if joint not in self.arm_joint_names:
            raise ValueError('%s not in arm joint list!' % joint)
        if joint not in self._joint_velocities.keys():
            raise ValueError('Joint velocity for joint'
                             ' $s not available!' % joint)
        return self._joint_velocities[joint]

    def get_joint_torque(self, joint):
        """
        Return the joint torque of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint torque
        :rtype: float
        """
        if joint not in self.arm_joint_names:
            raise ValueError('%s not in arm joint list!' % joint)
        if joint not in self._joint_efforts.keys():
            raise ValueError('Joint torque for joint $s'
                             ' not available!' % joint)
        return self._joint_efforts[joint]

    def set_joint_positions(self, positions, plan=True, wait=True, **kwargs):
        """
        Sets the desired joint angles for all arm joints

        :param positions: list of length #of joints, angles in radians
        :param plan: whether to use moveit to plan a path. Without planning,
                     there is no collision checking and each joint will
                     move to its target joint position directly.
        :param wait: if True, it will wait until the desired
                     joint positions are achieved
        :type positions: list
        :type plan: bool
        :type wait: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        result = False
        if isinstance(positions, np.ndarray):
            positions = positions.flatten().tolist()
        if plan:
            moveit_plan = self._compute_plan(positions)
            result = self._execute_plan(moveit_plan, wait=wait)
        else:
            self._pub_joint_positions(positions)
            if wait:
                result = self._loop_angle_pub_cmd(self._pub_joint_positions,
                                                  positions)
        return result

    def set_joint_velocities(self, velocities, **kwargs):
        """
        Sets the desired joint velocities for all arm joints

        :param velocities: target joint velocities
        :type velocities: list
        """
        self._pub_joint_velocities(velocities)

    def set_joint_torques(self, torques, **kwargs):
        """
        Sets the desired joint torques for all arm joints

        :param torques: target joint torques
        :type torques: list
        """
        self._pub_joint_torques(torques)

    def set_ee_pose(self, position, orientation, plan=True,
                    wait=True, numerical=True, **kwargs):
        """
        Commands robot arm to desired end-effector pose
        (w.r.t. 'ARM_BASE_FRAME').
        Computes IK solution in joint space and calls set_joint_positions.
        Will wait for command to complete if wait is set to True.

        :param position: position of the end effector (shape: :math:`[3,]`)
        :param orientation: orientation of the end effector
                            (can be rotation matrix, euler angles (yaw,
                            pitch, roll), or quaternion)
                            (shape: :math:`[3, 3]`, :math:`[3,]`
                            or :math:`[4,]`)
                            The convention of the Euler angles here
                            is z-y'-x' (intrinsic rotations),
                            which is one type of Tait-Bryan angles.
        :param plan: use moveit the plan a path to move to the desired pose
        :param wait: wait until the desired pose is achieved
        :param numerical: use numerical inverse kinematics solver or
                          analytical inverse kinematics solver
        :type position: np.ndarray
        :type orientation: np.ndarray
        :type plan: bool
        :type wait: bool
        :type numerical: bool
        :return: Returns True if command succeeded, False otherwise
        :rtype: bool
        """
        joint_positions = self.compute_ik(position, orientation,
                                          numerical=numerical)
        result = False
        if joint_positions is None:
            rospy.logerr('No IK solution found; check if target_pose is valid')
        else:
            result = self.set_joint_positions(joint_positions,
                                              plan=plan, wait=wait)
        return result

    def move_ee_xyz(self, displacement, eef_step=0.005,
                    numerical=True, plan=True, **kwargs):
        """
        Keep the current orientation fixed, move the end
        effector in {xyz} directions

        :param displacement: (delta_x, delta_y, delta_z)
        :param eef_step: resolution (m) of the interpolation
                         on the cartesian path
        :param numerical: use numerical inverse kinematics solver or
                          analytical inverse kinematics solver
        :param plan: use moveit the plan a path to move to the
                     desired pose. If False,
                     it will do linear interpolation along the path,
                     and simply use IK solver to find the
                     sequence of desired joint positions and
                     then call `set_joint_positions`
        :type displacement: np.ndarray
        :type eef_step: float
        :type numerical: bool
        :type plan: bool
        :return: whether the movement is successful or not
        :rtype: bool
        """
        displacement = displacement.reshape(-1, 1)

        path_len = np.linalg.norm(displacement)
        num_pts = int(np.ceil(path_len / float(eef_step)))
        if num_pts <= 1:
            num_pts = 2
        if (hasattr(self, 'ana_ik_solver') and not numerical) or not plan:
            ee_pose = self.get_ee_pose(self.configs.ARM.ARM_BASE_FRAME)
            cur_pos, cur_ori, cur_quat = ee_pose
            waypoints_sp = np.linspace(0, path_len, num_pts)
            waypoints = cur_pos + waypoints_sp / float(path_len) * displacement
            way_joint_positions = []
            qinit = self.get_joint_angles().tolist()
            for i in range(waypoints.shape[1]):
                joint_positions = self.compute_ik(waypoints[:, i].flatten(),
                                                  cur_quat,
                                                  qinit=qinit,
                                                  numerical=numerical)
                if joint_positions is None:
                    rospy.logerr('No IK solution found; '
                                 'check if target_pose is valid')
                    return False
                way_joint_positions.append(copy.deepcopy(joint_positions))
                qinit = copy.deepcopy(joint_positions)
            success = False
            for joint_positions in way_joint_positions:
                success = self.set_joint_positions(joint_positions,
                                                   plan=plan, wait=True)

            return success
        else:
            if not self.use_moveit:
                raise ValueError('Using plan=True when moveit is not'
                                 ' initialized, did you pass '
                                 'in use_moveit=True?')
            # ee_pose = self.get_ee_pose(self.configs.ARM.ARM_BASE_FRAME)
            # cur_pos, cur_ori, cur_quat = ee_pose
            cur_pose = self.moveit_group.get_current_pose()
            cur_pos = np.array([cur_pose.pose.position.x,
                                cur_pose.pose.position.y,
                                cur_pose.pose.position.z]).reshape(-1, 1)
            cur_quat = np.array([cur_pose.pose.orientation.x,
                                 cur_pose.pose.orientation.y,
                                 cur_pose.pose.orientation.z,
                                 cur_pose.pose.orientation.w])
            waypoints_sp = np.linspace(0, path_len, num_pts)
            waypoints = cur_pos + waypoints_sp / path_len * displacement
            moveit_waypoints = []
            wpose = self.moveit_group.get_current_pose().pose
            for i in range(waypoints.shape[1]):
                wpose.position.x = waypoints[0, i]
                wpose.position.y = waypoints[1, i]
                wpose.position.z = waypoints[2, i]
                wpose.orientation.x = cur_quat[0]
                wpose.orientation.y = cur_quat[1]
                wpose.orientation.z = cur_quat[2]
                wpose.orientation.w = cur_quat[3]
                moveit_waypoints.append(copy.deepcopy(wpose))
            (plan, fraction) = self.moveit_group.compute_cartesian_path(
                moveit_waypoints,  # waypoints to follow
                eef_step,  # eef_step
                0.0)  # jump_threshold
            self._execute_plan(plan, wait=True)
            cur_pose = self.moveit_group.get_current_pose()
            cur_pos = np.array([cur_pose.pose.position.x,
                                cur_pose.pose.position.y,
                                cur_pose.pose.position.z]).reshape(-1, 1)
            success = True
            diff = cur_pos.flatten() - waypoints[:, -1].flatten()
            error = np.linalg.norm(diff)
            if error > self.configs.ARM.MAX_EE_ERROR:
                rospy.logerr('Move end effector along xyz failed!')
                success = False
            return success

    def get_jacobian(self, joint_angles):
        """
        Return the geometric jacobian on the given joint angles.
        Refer to P112 in "Robotics: Modeling, Planning, and Control"

        :param joint_angles: joint_angles
        :type joint_angles: list or flattened np.ndarray
        :return: jacobian
        :rtype: np.ndarray
        """
        q = kdl.JntArray(self.urdf_chain.getNrOfJoints())
        for i in range(q.rows()):
            q[i] = joint_angles[i]
        jac = kdl.Jacobian(self.urdf_chain.getNrOfJoints())
        fg = self.jac_solver.JntToJac(q, jac)
        assert fg == 0, 'KDL JntToJac error!'
        jac_np = prutil.kdl_array_to_numpy(jac)
        return jac_np

    def compute_fk_position(self, joint_positions, des_frame):
        """
        Given joint angles, compute the pose of desired_frame with respect
        to the base frame (self.configs.ARM.ARM_BASE_FRAME). The desired frame
        must be in self.arm_link_names

        :param joint_positions: joint angles
        :param des_frame: desired frame
        :type joint_positions: np.ndarray
        :type des_frame: string
        :return: translational vector and rotational matrix
        :rtype: np.ndarray, np.ndarray
        """
        joint_positions = joint_positions.flatten()
        assert joint_positions.size == self.arm_dof
        kdl_jnt_angles = prutil.joints_to_kdl(joint_positions)

        kdl_end_frame = kdl.Frame()
        idx = self.arm_link_names.index(des_frame) + 1
        fg = self.fk_solver_pos.JntToCart(kdl_jnt_angles,
                                          kdl_end_frame,
                                          idx)
        assert fg == 0, 'KDL Pos JntToCart error!'
        pose = prutil.kdl_frame_to_numpy(kdl_end_frame)
        pos = pose[:3, 3].reshape(-1, 1)
        rot = pose[:3, :3]
        return pos, rot

    def compute_fk_velocity(self, joint_positions,
                            joint_velocities, des_frame):
        """
        Given joint_positions and joint velocities,
        compute the velocities of des_frame with respect
        to the base frame

        :param joint_positions: joint positions
        :param joint_velocities: joint velocities
        :param des_frame: end frame
        :type joint_positions: np.ndarray
        :type joint_velocities: np.ndarray
        :type des_frame: string
        :return: translational and rotational
                 velocities (vx, vy, vz, wx, wy, wz)
        :rtype: np.ndarray
        """
        kdl_end_frame = kdl.FrameVel()
        kdl_jnt_angles = prutil.joints_to_kdl(joint_positions)
        kdl_jnt_vels = prutil.joints_to_kdl(joint_velocities)
        kdl_jnt_qvels = kdl.JntArrayVel(kdl_jnt_angles, kdl_jnt_vels)
        idx = self.arm_link_names.index(des_frame) + 1
        fg = self.fk_solver_vel.JntToCart(kdl_jnt_qvels,
                                          kdl_end_frame,
                                          idx)
        assert fg == 0, 'KDL Vel JntToCart error!'

        end_twist = kdl_end_frame.GetTwist()
        return np.array([end_twist[0], end_twist[1], end_twist[2],
                         end_twist[3], end_twist[4], end_twist[5]])

    def compute_ik(self, position, orientation, qinit=None, numerical=True):
        """
        Inverse kinematics

        :param position: end effector position (shape: :math:`[3,]`)
        :param orientation: end effector orientation.
                            It can be quaternion ([x,y,z,w],
                            shape: :math:`[4,]`),
                            euler angles (yaw, pitch, roll
                            angles (shape: :math:`[3,]`)),
                            or rotation matrix (shape: :math:`[3, 3]`)
        :param qinit: initial joint positions for numerical IK
        :param numerical: use numerical IK or analytical IK
        :type position: np.ndarray
        :type orientation: np.ndarray
        :type qinit: list
        :type numerical: bool
        :return: None or joint positions
        :rtype: np.ndarray
        """
        position = position.flatten()
        if orientation.size == 4:
            orientation = orientation.flatten()
            ori_x = orientation[0]
            ori_y = orientation[1]
            ori_z = orientation[2]
            ori_w = orientation[3]
        elif orientation.size == 3:
            quat = prutil.euler_to_quat(orientation)
            ori_x = quat[0]
            ori_y = quat[1]
            ori_z = quat[2]
            ori_w = quat[3]
        elif orientation.size == 9:
            quat = prutil.rot_mat_to_quat(orientation)
            ori_x = quat[0]
            ori_y = quat[1]
            ori_z = quat[2]
            ori_w = quat[3]
        else:
            raise TypeError('Orientation must be in one '
                            'of the following forms:'
                            'rotation matrix, euler angles, or quaternion')
        if qinit is None:
            qinit = self.get_joint_angles().tolist()
        elif isinstance(qinit, np.ndarray):
            qinit = qinit.flatten().tolist()
        if numerical:
            pos_tol = self.configs.ARM.IK_POSITION_TOLERANCE
            ori_tol = self.configs.ARM.IK_ORIENTATION_TOLERANCE
            joint_positions = self.num_ik_solver.get_ik(qinit,
                                                        position[0],
                                                        position[1],
                                                        position[2],
                                                        ori_x,
                                                        ori_y,
                                                        ori_z,
                                                        ori_w,
                                                        pos_tol,
                                                        pos_tol,
                                                        pos_tol,
                                                        ori_tol,
                                                        ori_tol,
                                                        ori_tol)
        else:
            if not hasattr(self, 'ana_ik_solver'):
                raise TypeError('Analytical solver not provided, '
                                'please use `numerical=True`'
                                'to use the numerical method '
                                'for inverse kinematics')
            else:
                joint_positions = self.ana_ik_solver.get_ik(qinit,
                                                            position[0],
                                                            position[1],
                                                            position[2],
                                                            ori_x,
                                                            ori_y,
                                                            ori_z,
                                                            ori_w)
        if joint_positions is None:
            return None
        return np.array(joint_positions)

    def _get_kdl_link_names(self):
        num_links = self.urdf_chain.getNrOfSegments()
        link_names = []
        for i in range(num_links):
            link_names.append(self.urdf_chain.getSegment(i).getName())
        return copy.deepcopy(link_names)

    def _get_kdl_joint_names(self):
        num_links = self.urdf_chain.getNrOfSegments()
        num_joints = self.urdf_chain.getNrOfJoints()
        joint_names = []
        for i in range(num_links):
            link = self.urdf_chain.getSegment(i)
            joint = link.getJoint()
            joint_type = joint.getType()
            # JointType definition: [RotAxis,RotX,RotY,RotZ,TransAxis,
            #                        TransX,TransY,TransZ,None]
            if joint_type > 1:
                continue
            joint_names.append(joint.getName())
        assert num_joints == len(joint_names)
        return copy.deepcopy(joint_names)

    def _callback_joint_states(self, msg):
        """
        ROS subscriber callback for arm joint state (position, velocity)

        :param msg: Contains message published in topic
        :type msg: sensor_msgs/JointState
        """
        self.joint_state_lock.acquire()
        for idx, name in enumerate(msg.name):
            if name in self.arm_joint_names:
                if idx < len(msg.position):
                    self._joint_angles[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._joint_velocities[name] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self._joint_efforts[name] = msg.effort[idx]
        self.joint_state_lock.release()

    def _pub_joint_positions(self, positions):
        joint_state = JointState()
        joint_state.position = tuple(positions)
        self.joint_pub.publish(joint_state)

    def _pub_joint_velocities(self, velocities):
        joint_state = JointState()
        joint_state.velocity = tuple(velocities)
        self.joint_pub.publish(joint_state)

    def _pub_joint_torques(self, torques):
        joint_state = JointState()
        joint_state.effort = tuple(torques)
        self.joint_pub.publish(joint_state)

    def _init_moveit(self):
        """
        Initialize moveit and setup move_group object
        """
        moveit_commander.roscpp_initialize(sys.argv)
        mg_name = self.configs.ARM.MOVEGROUP_NAME
        self.moveit_group = moveit_commander.MoveGroupCommander(mg_name)
        self.moveit_group.set_planner_id(self.moveit_planner)
        self.scene = moveit_commander.PlanningSceneInterface()

    def _compute_plan(self, target_joint):
        """
        Computes motion plan to achieve desired target_joint

        :param target_joint: list of length #of joints, angles in radians
        :type target_joint: np.ndarray

        :return: Computed motion plan
        :rtype: moveit_msgs.msg.RobotTrajectory
        """
        # TODO Check if target_joint is valid
        if not self.use_moveit:
            raise ValueError('Moveit is not initialized, '
                             'did you pass in use_moveit=True?')
        if isinstance(target_joint, np.ndarray):
            target_joint = target_joint.tolist()
        self.moveit_group.set_joint_value_target(target_joint)
        rospy.loginfo('Moveit Motion Planning...')
        return self.moveit_group.plan()

    def _execute_plan(self, plan, wait=True):
        """
        Executes plan on arm controller

        :param plan: motion plan to execute
        :param wait: True if blocking call and will return after
                     target_joint is achieved, False otherwise
        :type plan: moveit_msgs.msg.RobotTrajectory
        :type wait: bool

        :return: True if arm executed plan, False otherwise
        :rtype: bool
        """
        result = False
        if not self.use_moveit:
            raise ValueError('Moveit is not initialized, '
                             'did you pass in use_moveit=True?')
        if len(plan.joint_trajectory.points) < 1:
            rospy.logwarn('No motion plan found. No execution attempted')
        else:
            rospy.loginfo('Executing...')
            result = self.moveit_group.execute(plan, wait=wait)
        return result

    def _angle_error_is_small(self, target_joints):
        cur_joint_vals = self.get_joint_angles()
        joint_diff = cur_joint_vals - np.array(target_joints)
        error = np.max(np.abs(joint_diff))
        if error <= self.configs.ARM.MAX_JOINT_ERROR:
            return joint_diff, error, True
        else:
            return joint_diff, error, False

    def _loop_angle_pub_cmd(self, cmd, joint_vals):
        start_time = time.time()
        vel_zero_times = 0
        # wait 10s in worse case
        for i in range(int(10 / self.configs.ARM.WAIT_MIN_TIME)):
            cmd(joint_vals)
            res = self._angle_error_is_small(joint_vals)
            joint_diff, error, eflag = res
            if eflag:
                success = True
                break
            vels = self.get_joint_velocities()
            es_time = time.time() - start_time
            if es_time > 0.5 and np.max(np.abs(vels)) < 0.01:
                vel_zero_times += 1
            else:
                vel_zero_times = 0
            if vel_zero_times > 10:
                success = False
                break
            rospy.sleep(self.configs.ARM.WAIT_MIN_TIME)
        return success

    def _setup_joint_pub(self):
        self.joint_pub = rospy.Publisher(
            self.configs.ARM.ROSTOPIC_SET_JOINT, JointState, queue_size=1)

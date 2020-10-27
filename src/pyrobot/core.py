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

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import JointState, CameraInfo, Image

import pyrobot.utils.util as prutil

from pyrobot.utils.move_group_interface import MoveGroupInterface as MoveGroup
from pyrobot_bridge.srv import *

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError

import message_filters

import actionlib
from pyrobot_bridge.msg import (
    MoveitAction,
    MoveitGoal,
)
from actionlib_msgs.msg import GoalStatus


class Robot:
    """
    This is the main interface class that is composed of
    key robot modules (base, arm, gripper, and camera).
    This class builds robot specific objects by reading a
    configuration and instantiating the necessary robot module objects.

    """

    def __init__(
        self,
        robot_name,
        use_arm=True,
        use_base=True,
        use_camera=True,
        use_gripper=True,
        arm_config={},
        base_config={},
        camera_config={},
        gripper_config={},
        common_config={},
    ):
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
        cfg_path = os.path.join(root_path, "cfg")
        robot_pool = []
        for f in os.listdir(cfg_path):
            if f.endswith("_config.py"):
                robot_pool.append(f[: -len("_config.py")])
        root_node = "pyrobot."
        self.configs = None
        this_robot = None
        for srobot in robot_pool:
            if srobot in robot_name:
                this_robot = srobot
                mod = importlib.import_module(
                    root_node + "cfg." + "{:s}_config".format(srobot)
                )
                cfg_func = getattr(mod, "get_cfg")
                if srobot == "locobot" and "lite" in robot_name:
                    self.configs = cfg_func("create")
                else:
                    self.configs = cfg_func()
        if self.configs is None:
            raise ValueError(
                "Invalid robot name provided, only the following"
                " are currently available: {}".format(robot_pool)
            )
        self.configs.freeze()
        try:
            rospy.init_node("pyrobot", anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn("ROS node [pyrobot] has already been initialized")

        root_node += this_robot
        root_node += "."
        if self.configs.HAS_COMMON:
            mod = importlib.import_module(root_node + self.configs.COMMON.NAME)
            common_class = getattr(mod, self.configs.COMMON.CLASS)
            setattr(
                self,
                self.configs.COMMON.NAME,
                common_class(self.configs, **common_config),
            )
        if self.configs.HAS_ARM and use_arm:
            mod = importlib.import_module(root_node + "arm")
            arm_class = getattr(mod, self.configs.ARM.CLASS)
            if self.configs.HAS_COMMON:
                arm_config[self.configs.COMMON.NAME] = getattr(
                    self, self.configs.COMMON.NAME
                )
            self.arm = arm_class(self.configs, **arm_config)
        if self.configs.HAS_BASE and use_base:
            mod = importlib.import_module(root_node + "base")
            base_class = getattr(mod, self.configs.BASE.CLASS)
            if self.configs.HAS_COMMON:
                base_config[self.configs.COMMON.NAME] = getattr(
                    self, self.configs.COMMON.NAME
                )
            self.base = base_class(self.configs, **base_config)
        if self.configs.HAS_CAMERA and use_camera:
            mod = importlib.import_module(root_node + "camera")
            camera_class = getattr(mod, self.configs.CAMERA.CLASS)
            if self.configs.HAS_COMMON:
                camera_config[self.configs.COMMON.NAME] = getattr(
                    self, self.configs.COMMON.NAME
                )
            self.camera = camera_class(self.configs, **camera_config)
        if self.configs.HAS_GRIPPER and use_gripper and use_arm:
            mod = importlib.import_module(root_node + "gripper")
            gripper_class = getattr(mod, self.configs.GRIPPER.CLASS)
            if self.configs.HAS_COMMON:
                gripper_config[self.configs.COMMON.NAME] = getattr(
                    self, self.configs.COMMON.NAME
                )
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
        self.ctrl_pub = rospy.Publisher(
            configs.BASE.ROSTOPIC_BASE_COMMAND, Twist, queue_size=1
        )

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
            rospy.sleep(1.0 / self.configs.BASE.BASE_CONTROL_RATE)

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

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
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

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
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

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
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
        self.cv_bridge = CvBridge()
        self.camera_info_lock = threading.RLock()
        self.camera_img_lock = threading.RLock()
        self.rgb_img = None
        self.depth_img = None
        self.camera_info = None
        self.camera_P = None
        rospy.Subscriber(
            self.configs.CAMERA.ROSTOPIC_CAMERA_INFO_STREAM,
            CameraInfo,
            self._camera_info_callback,
        )

        rgb_topic = self.configs.CAMERA.ROSTOPIC_CAMERA_RGB_STREAM
        self.rgb_sub = message_filters.Subscriber(rgb_topic, Image)
        depth_topic = self.configs.CAMERA.ROSTOPIC_CAMERA_DEPTH_STREAM
        self.depth_sub = message_filters.Subscriber(depth_topic, Image)
        img_subs = [self.rgb_sub, self.depth_sub]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            img_subs, queue_size=10, slop=0.2
        )
        self.sync.registerCallback(self._sync_callback)

    def _sync_callback(self, rgb, depth):
        self.camera_img_lock.acquire()
        try:
            self.rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")
            self.rgb_img = self.rgb_img[:, :, ::-1]
            self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.camera_img_lock.release()

    def _camera_info_callback(self, msg):
        self.camera_info_lock.acquire()
        self.camera_info = msg
        self.camera_P = np.array(msg.P).reshape((3, 4))
        self.camera_info_lock.release()

    def get_rgb(self):
        """
        This function returns the RGB image perceived by the camera.

        :rtype: np.ndarray or None
        """
        self.camera_img_lock.acquire()
        rgb = copy.deepcopy(self.rgb_img)
        self.camera_img_lock.release()
        return rgb

    def get_depth(self):
        """
        This function returns the depth image perceived by the camera.
        
        The depth image is in meters.
        
        :rtype: np.ndarray or None
        """
        self.camera_img_lock.acquire()
        depth = copy.deepcopy(self.depth_img)
        self.camera_img_lock.release()
        depth = depth / self.configs.CAMERA.DEPTH_MAP_FACTOR
        return depth

    def get_rgb_depth(self):
        """
        This function returns both the RGB and depth
        images perceived by the camera. 

        The depth image is in meters.

        :rtype: np.ndarray or None
        """
        self.camera_img_lock.acquire()
        rgb = copy.deepcopy(self.rgb_img)
        depth = copy.deepcopy(self.depth_img)
        depth = depth / self.configs.CAMERA.DEPTH_MAP_FACTOR
        self.camera_img_lock.release()
        return rgb, depth

    def get_intrinsics(self):
        """
        This function returns the camera intrinsics.

        :rtype: np.ndarray
        """
        if self.camera_P is None:
            return self.camera_P
        self.camera_info_lock.acquire()
        P = copy.deepcopy(self.camera_P)
        self.camera_info_lock.release()
        return P[:3, :3]


class Arm(object):
    """
    This is a parent class on which the robot
    specific Arm classes would be built.
    """

    __metaclass__ = ABCMeta

    def __init__(
        self,
        configs,
        moveit_planner="ESTkConfigDefault",
        planning_time=30,
        analytical_ik=None,
        use_moveit=True,
    ):
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
        self.planning_time = planning_time

        self.use_moveit = use_moveit

        self.joint_state_lock = threading.RLock()
        self.tf_listener = tf.TransformListener()
        if self.use_moveit:
            self._init_moveit()

        if analytical_ik is not None:
            self.ana_ik_solver = analytical_ik(
                configs.ARM.ARM_BASE_FRAME, configs.ARM.EE_FRAME
            )

        self.arm_joint_names = self.configs.ARM.JOINT_NAMES
        self.arm_dof = len(self.arm_joint_names)

        # Subscribers
        self._joint_angles = dict()
        self._joint_velocities = dict()
        self._joint_efforts = dict()
        rospy.Subscriber(
            configs.ARM.ROSTOPIC_JOINT_STATES, JointState, self._callback_joint_states
        )

        # Ros-Params
        rospy.set_param("pyrobot/base_link", configs.ARM.ARM_BASE_FRAME)
        rospy.set_param("pyrobot/gripper_link", configs.ARM.EE_FRAME)
        rospy.set_param(
            "pyrobot/robot_description", configs.ARM.ARM_ROBOT_DSP_PARAM_NAME
        )

        # Publishers
        self.joint_pub = None
        self._setup_joint_pub()

        # Services
        self._ik_service = rospy.ServiceProxy("pyrobot/ik", IkCommand)
        try:
            self._ik_service.wait_for_service(timeout=3)
        except:
            rospy.logerr("Timeout waiting for Inverse Kinematics Service!!")

        self._fk_service = rospy.ServiceProxy("pyrobot/fk", FkCommand)
        try:
            self._fk_service.wait_for_service(timeout=3)
        except:
            rospy.logerr("Timeout waiting for Forward Kinematics Service!!")

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
        trans, quat = prutil.get_tf_transform(self.tf_listener, src_frame, dest_frame)
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
                rospy.loginfo("Torque value for joint " "[%s] not available!" % joint)
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
            raise ValueError("%s not in arm joint list!" % joint)
        if joint not in self._joint_angles.keys():
            raise ValueError("Joint angle for joint $s not available!" % joint)
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
            raise ValueError("%s not in arm joint list!" % joint)
        if joint not in self._joint_velocities.keys():
            raise ValueError("Joint velocity for joint" " $s not available!" % joint)
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
            raise ValueError("%s not in arm joint list!" % joint)
        if joint not in self._joint_efforts.keys():
            raise ValueError("Joint torque for joint $s" " not available!" % joint)
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
            if not self.use_moveit:
                raise ValueError(
                    "Moveit is not initialized, " "did you pass in use_moveit=True?"
                )

            self._cancel_moveit_goal()

            if isinstance(positions, np.ndarray):
                positions = positions.tolist()

            rospy.loginfo("Moveit Motion Planning...")

            moveit_goal = MoveitGoal()
            moveit_goal.wait = wait
            moveit_goal.action_type = "set_joint_positions"
            moveit_goal.values = positions
            self._moveit_client.send_goal(moveit_goal)

            if wait:
                self._moveit_client.wait_for_result()
                status = self._moveit_client.get_state()
                if status == GoalStatus.SUCCEEDED:
                    result = True
        else:
            self._pub_joint_positions(positions)
            if wait:
                result = self._loop_angle_pub_cmd(self._pub_joint_positions, positions)

        if wait:
            return result

    def make_plan_joint_positions(self, positions, **kwargs):
        result = None
        if isinstance(positions, np.ndarray):
            positions = positions.flatten().tolist()

        if not self.use_moveit:
            raise ValueError(
                "Moveit is not initialized, " "did you pass in use_moveit=True?"
            )

        rospy.loginfo("Moveit Motion Planning...")

        raise NotImplementedError
        # result = self.moveit_group.motionPlanToJointPosition(
        #     self.arm_joint_names, positions
        # )
        # return [p.positions for p in result]

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

    def set_ee_pose(
        self, position, orientation, plan=True, wait=True, numerical=True, **kwargs
    ):
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
        joint_positions = self.compute_ik(position, orientation, numerical=numerical)
        result = False
        if joint_positions is None:
            rospy.logerr("No IK solution found; check if target_pose is valid")
        else:
            result = self.set_joint_positions(joint_positions, plan=plan, wait=wait)
        return result

    def make_plan_pose(
        self, position, orientation, wait=True, numerical=True, **kwargs
    ):

        if not self.use_moveit:
            raise ValueError(
                "Using plan=True when moveit is not"
                " initialized, did you pass "
                "in use_moveit=True?"
            )
        pose = Pose()
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
            raise TypeError(
                "Orientation must be in one "
                "of the following forms:"
                "rotation matrix, euler angles, or quaternion"
            )
        pose.position.x, pose.position.y, pose.position.z = (
            position[0],
            position[1],
            position[2],
        )
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = (ori_x, ori_y, ori_z, ori_w)

        raise NotImplementedError
        # result = self.moveit_group.motionPlanToPose(pose, wait=True)
        # return [p.positions for p in result]

    def move_ee_xyz(
        self,
        displacement,
        eef_step=0.005,
        numerical=True,
        plan=True,
        wait=True,
        **kwargs
    ):
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
        if (hasattr(self, "ana_ik_solver") and not numerical) or not plan:

            if not wait:
                raise NotImplementedError(
                    "Not wait is supported only in plan=True case."
                )

            ee_pose = self.get_ee_pose(self.configs.ARM.ARM_BASE_FRAME)
            cur_pos, cur_ori, cur_quat = ee_pose
            waypoints_sp = np.linspace(0, path_len, num_pts)
            waypoints = cur_pos + waypoints_sp / float(path_len) * displacement
            way_joint_positions = []
            qinit = self.get_joint_angles().tolist()
            for i in range(waypoints.shape[1]):
                joint_positions = self.compute_ik(
                    waypoints[:, i].flatten(),
                    cur_quat,
                    qinit=qinit,
                    numerical=numerical,
                )
                if joint_positions is None:
                    rospy.logerr(
                        "No IK solution found; " "check if target_pose is valid"
                    )
                    return False
                way_joint_positions.append(copy.deepcopy(joint_positions))
                qinit = copy.deepcopy(joint_positions)
            success = False
            for joint_positions in way_joint_positions:
                success = self.set_joint_positions(
                    joint_positions, plan=plan, wait=True
                )

            return success
        else:
            if not self.use_moveit:
                raise ValueError(
                    "Using plan=True when moveit is not"
                    " initialized, did you pass "
                    "in use_moveit=True?"
                )

            self._cancel_moveit_goal()

            ee_pose = self.get_ee_pose(self.configs.ARM.ARM_BASE_FRAME)
            cur_pos, cur_ori, cur_quat = ee_pose
            cur_pos = np.array(cur_pos).reshape(-1, 1)
            cur_quat = np.array(cur_quat)

            waypoints_sp = np.linspace(0, path_len, num_pts)
            waypoints = cur_pos + waypoints_sp / path_len * displacement
            moveit_waypoints = []
            wpose = Pose()
            for i in range(waypoints.shape[1]):
                if i < 1:
                    continue
                wpose.position.x = waypoints[0, i]
                wpose.position.y = waypoints[1, i]
                wpose.position.z = waypoints[2, i]
                wpose.orientation.x = cur_quat[0]
                wpose.orientation.y = cur_quat[1]
                wpose.orientation.z = cur_quat[2]
                wpose.orientation.w = cur_quat[3]
                moveit_waypoints.append(copy.deepcopy(wpose))

            moveit_goal = MoveitGoal()
            moveit_goal.wait = wait
            moveit_goal.action_type = "move_ee_xyz"
            moveit_goal.waypoints = moveit_waypoints
            moveit_goal.eef_step = eef_step
            self._moveit_client.send_goal(moveit_goal)

            if wait:
                self._moveit_client.wait_for_result()
                status = self._moveit_client.get_state()
                if status != GoalStatus.SUCCEEDED:
                    return False

                ee_pose = self.get_ee_pose(self.configs.ARM.ARM_BASE_FRAME)
                cur_pos, cur_ori, cur_quat = ee_pose

                cur_pos = np.array(cur_pos).reshape(-1, 1)

                success = True
                diff = cur_pos.flatten() - waypoints[:, -1].flatten()
                error = np.linalg.norm(diff)
                if error > self.configs.ARM.MAX_EE_ERROR:
                    rospy.logerr("Move end effector along xyz failed!")
                    success = False
                return success

    def _cancel_moveit_goal(self):
        if not self._moveit_client.gh:
            return
        # 2 is for "Done" state of action
        if self._moveit_client.simple_state != 2:
            self._moveit_client.cancel_all_goals()

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

        req = FkCommandRequest()
        req.joint_angles = list(joint_positions)
        req.end_frame = des_frame
        try:
            resp = self._fk_service(req)
        except:
            rospy.logerr("FK Service call failed")
            resp = FkCommandResponse()
            resp.success = False

        if not resp.success:
            return None

        pos = np.asarray(resp.pos).reshape(3, 1)
        rot = prutil.quat_to_rot_mat(resp.quat)
        return pos, rot

    def get_jacobian(self, joint_angles):
        """
        Return the geometric jacobian on the given joint angles.
        Refer to P112 in "Robotics: Modeling, Planning, and Control"

        :param joint_angles: joint_angles
        :type joint_angles: list or flattened np.ndarray
        :return: jacobian
        :rtype: np.ndarray
        """
        raise NotImplementedError

    def compute_fk_velocity(self, joint_positions, joint_velocities, des_frame):
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
        raise NotImplementedError

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
            raise TypeError(
                "Orientation must be in one "
                "of the following forms:"
                "rotation matrix, euler angles, or quaternion"
            )
        if qinit is None:
            qinit = self.get_joint_angles().tolist()
        elif isinstance(qinit, np.ndarray):
            qinit = qinit.flatten().tolist()
        if numerical:
            pos_tol = self.configs.ARM.IK_POSITION_TOLERANCE
            ori_tol = self.configs.ARM.IK_ORIENTATION_TOLERANCE

            req = IkCommandRequest()
            req.init_joint_positions = qinit
            req.pose = [
                position[0],
                position[1],
                position[2],
                ori_x,
                ori_y,
                ori_z,
                ori_w,
            ]
            req.tolerance = 3 * [pos_tol] + 3 * [ori_tol]

            try:
                resp = self._ik_service(req)
            except:
                rospy.logerr("IK Service call failed")
                resp = IkCommandResponse()
                resp.success = False

            if not resp.success:
                joint_positions = None
            else:
                joint_positions = resp.joint_positions
        else:
            if not hasattr(self, "ana_ik_solver"):
                raise TypeError(
                    "Analytical solver not provided, "
                    "please use `numerical=True`"
                    "to use the numerical method "
                    "for inverse kinematics"
                )
            else:
                joint_positions = self.ana_ik_solver.get_ik(
                    qinit,
                    position[0],
                    position[1],
                    position[2],
                    ori_x,
                    ori_y,
                    ori_z,
                    ori_w,
                )
        if joint_positions is None:
            return None
        print(joint_positions)
        return np.array(joint_positions)

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
        rospy.set_param("pyrobot/moveit_planner", self.moveit_planner)
        rospy.set_param("pyrobot/move_group_name", self.configs.ARM.MOVEGROUP_NAME)
        self._moveit_client = actionlib.SimpleActionClient(
            "/pyrobot/moveit_server", MoveitAction
        )

        rospy.sleep(0.1)  # Ensures client spins up properly
        server_up = self._moveit_client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr(
                "Timed out waiting for the pyrobot-moveit server"
                " Action Server to connect. Start the action server"
                " before running example."
            )
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

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
        success = False
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
            self.configs.ARM.ROSTOPIC_SET_JOINT, JointState, queue_size=1
        )

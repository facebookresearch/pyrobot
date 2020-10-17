#!/usr/bin/env python
# coding: utf-8

from config import get_cfg_defaults

_C = get_cfg_defaults()

_ARMC = _C.ARM
# Arm class name
_ARMC.CLASS = 'PepperArm'
# robot description parameter name for the arm
_C.ARM.ARM_ROBOT_DSP_PARAM_NAME = '/robot_description'
# Right arm movegroup name
_ARMC.MOVEGROUP_NAME = 'right_arm'
# Left arm movegroup name
_ARMC.EXTRA_MOVEGROUP_NAME = 'left_arm'
# Arm base frame name
_ARMC.ARM_BASE_FRAME = 'torso'
# Arm end effector frame name (right arm)
_ARMC.EE_FRAME = 'r_gripper'
# Arm end effector frame name (left arm)
_ARMC.EXTRA_EE_FRAME = 'l_gripper'
# minimum waiting time to check again if the set command is complete
_C.ARM.WAIT_MIN_TIME = 0.05
# Joint states topic name
_ARMC.ROSTOPIC_JOINT_STATES = '/joint_states'
# Joint angles topic name
_ARMC.ROSTOPIC_SET_JOINT = '/joint_angles'
# maximum allowed joint error for checking if the target joint value is reached
_C.ARM.MAX_JOINT_ERROR = 0.2
# Names of the joints in the right arm kinematic chain
_ARMC.JOINT_NAMES = [
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw"]
# Names of the joints in the left arm kinematic chain
_ARMC.EXTRA_JOINT_NAMES = [
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw"]

_CAMERAC = _C.CAMERA
# Camera class name
_CAMERAC.CLASS = 'PepperCamera'
# CameraInfo topic for the top camera
_CAMERAC.ROSTOPIC_CAMERA_TOP_INFO_STREAM = '/naoqi_driver/camera/front/camera_info'
# Image topic for the top camera
_CAMERAC.ROSTOPIC_CAMERA_TOP_STREAM = '/naoqi_driver/camera/front/image_raw'
# CameraInfo topic for the bottom camera
_CAMERAC.ROSTOPIC_CAMERA_BOTTOM_INFO_STREAM = '/naoqi_driver/camera/bottom/camera_info'
# Image topic for the bottom camera
_CAMERAC.ROSTOPIC_CAMERA_BOTTOM_STREAM = '/naoqi_driver/camera/bottom/image_raw'
# CameraInfo topic for the depth camera
_CAMERAC.ROSTOPIC_CAMERA_DEPTH_INFO_STREAM = '/naoqi_driver/camera/depth/camera_info'
# Image topic for the depth camera
_CAMERAC.ROSTOPIC_CAMERA_DEPTH_STREAM = 'naoqi_driver/camera/depth/image_raw'

_BASEC = _C.BASE
# Base class name
_BASEC.CLASS = 'PepperBase'
# Rostopic used to send a velocity command to the robot's base
_BASEC.ROSTOPIC_BASE_COMMAND = '/cmd_vel'
# Rostopic used to send a position command to the robot's base
_BASEC.ROSTOPIC_BASE_POS_COMMAND = '/move_base_simple/goal'
# Rostopic on which the wheel-encoder-based odommetry is available
_BASEC.ROSTOPIC_ODOMETRY = '/naoqi_driver/odom'
# Rostopic on which base bumper sensor informations is available
_BASEC.ROSTOPIC_BUMPER = '/naoqi_driver/bumper'
# Control rate of the base
_BASEC.BASE_CONTROL_RATE = 10
# Maximum planar velocity for the robot's base
_BASEC.MAX_ABS_FWD_SPEED = 0.55
# Maximum angular velocity for the robot's base
_BASEC.MAX_ABS_TURN_SPEED = 2.0

_GRIPPERC = _C.GRIPPER
# Gripper class name
_GRIPPERC.CLASS = 'PepperGripper'
# Right gripper joint name
_GRIPPERC.R_GRIPPER_JOINT_NAME = "RHand"
# Left gripper joint name
_GRIPPERC.L_GRIPPER_JOINT_NAME = "LHand"
# Gripper opened position
_GRIPPERC.GRIPPER_OPENED_POSITION = 1.0
# Gripper closed position
_GRIPPERC.GRIPPER_CLOSED_POSITION = 0.0
# Joint angles topic name
_GRIPPERC.ROSTOPIC_SET_JOINT = "/joint_angles"


def get_cfg():
    return _C.clone()

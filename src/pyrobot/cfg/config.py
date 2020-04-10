# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from yacs.config import CfgNode as CN

_C = CN()

# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a mobile base or not
_C.HAS_BASE = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a gripper or not
_C.HAS_GRIPPER = True
# whether the robot has a common shared class among all components
_C.HAS_COMMON = False

_C.ARM = CN()
# ARM class name
_C.ARM.CLASS = "LoCoBotArm"
# robot description parameter name for the arm
_C.ARM.ARM_ROBOT_DSP_PARAM_NAME = "/robot_description"
# moveit group name for the arm
_C.ARM.MOVEGROUP_NAME = "arm"
# base frame for the arm
_C.ARM.ARM_BASE_FRAME = "base_link"
# end-effector frame of the arm
_C.ARM.EE_FRAME = "gripper_link"
# inverse kinematics position tolerance (m)
_C.ARM.IK_POSITION_TOLERANCE = 0.01
# inverse kinematics orientation toelrance (rad)
_C.ARM.IK_ORIENTATION_TOLERANCE = 0.1
# maximum allowed joint error for checking if the target joint value is reached
_C.ARM.MAX_JOINT_ERROR = 0.09
# maximum allowed end-effector position error in `move_ee_xyz`
_C.ARM.MAX_EE_ERROR = 0.03
# minimum waiting time to check again if the set command is complete
_C.ARM.WAIT_MIN_TIME = 0.05
# joint states topic name for the arm
_C.ARM.ROSTOPIC_JOINT_STATES = "/joint_states"
# topic name to set joint position/velocity/torque
_C.ARM.ROSTOPIC_SET_JOINT = "/goal_dynamixel_position"

_C.ARM.ROSSRV_CART_PATH = "/compute_cartesian_path"

_C.ARM.ROSSRV_MP_PATH = "/plan_kinematic_path"

_C.CAMERA = CN()

_C.BASE = CN()

_C.GRIPPER = CN()

_C.COMMON = CN()


def get_cfg_defaults():
    return _C.clone()

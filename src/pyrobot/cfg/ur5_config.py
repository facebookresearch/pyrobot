# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from pyrobot.cfg.config import get_cfg_defaults

_C = get_cfg_defaults()

_C.HAS_BASE = False
_C.HAS_CAMERA = False
_C.HAS_GRIPPER = False

_ARMC = _C.ARM
_ARMC.CLASS = "UR5Arm"
_ARMC.MOVEGROUP_NAME = "manipulator"
_ARMC.ARM_BASE_FRAME = "base_link"
_ARMC.EE_FRAME = "ee_link"
_ARMC.ROSTOPIC_JOINT_STATES = "/joint_states"
_ARMC.JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

_ARMC.ARM_MAX_JOINT_VELOCITY = 0.6

# ur5 only allows setting joint angles through moveit
# _ARMC.ROSTOPIC_SET_JOINT = '/robot/limb/right/joint_command'

_ARMC.IK_POSITION_TOLERANCE = 0.005
_ARMC.IK_ORIENTATION_TOLERANCE = 0.005

# _GRIPPERC = _C.GRIPPER
# # GRIPPER class name
# _GRIPPERC.CLASS = 'ur5Gripper'
# # maximum gripper open position
# _GRIPPERC.GRIPPER_MAX_POSITION = 0.041667
# # minimum gripper open position
# _GRIPPERC.GRIPPER_MIN_POSITION = 0.0
# # maximum gripper movement velocity
# _GRIPPERC.GRIPPER_MAX_VELOCITY = 3.0
# # minimum gripper movement velocity
# _GRIPPERC.GRIPPER_MIN_VELOCITY = 0.15


def get_cfg():
    return _C.clone()

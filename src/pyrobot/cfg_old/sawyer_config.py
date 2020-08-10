# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from pyrobot.cfg.config import get_cfg_defaults

_C = get_cfg_defaults()

_C.HAS_BASE = False
_C.HAS_CAMERA = False

_ARMC = _C.ARM
_ARMC.CLASS = "SawyerArm"
_ARMC.MOVEGROUP_NAME = "right_arm"
_ARMC.ARM_BASE_FRAME = "right_arm_base_link"
_ARMC.EE_FRAME = "right_gripper_tip"
_ARMC.ROSTOPIC_JOINT_STATES = "/robot/joint_states"
_ARMC.ROSTOPIC_SET_JOINT = "/robot/limb/right/joint_command"
_ARMC.ROSTOPIC_COLLISION_STATE = "/robot/limb/right/collision_detection_state"
_ARMC.JOINT_NAMES = [
    "right_j0",
    "right_j1",
    "right_j2",
    "right_j3",
    "right_j4",
    "right_j5",
    "right_j6",
]
_ARMC.IK_POSITION_TOLERANCE = 0.005
_ARMC.IK_ORIENTATION_TOLERANCE = 0.005

_GRIPPERC = _C.GRIPPER
# GRIPPER class name
_GRIPPERC.CLASS = "SawyerGripper"
# maximum gripper open position
_GRIPPERC.GRIPPER_MAX_POSITION = 0.041667
# minimum gripper open position
_GRIPPERC.GRIPPER_MIN_POSITION = 0.0
# maximum gripper movement velocity
_GRIPPERC.GRIPPER_MAX_VELOCITY = 3.0
# minimum gripper movement velocity
_GRIPPERC.GRIPPER_MIN_VELOCITY = 0.15


def get_cfg():
    return _C.clone()

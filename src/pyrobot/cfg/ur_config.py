# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from config import get_cfg_defaults

_C = get_cfg_defaults()

_C.HAS_BASE = False
_C.HAS_CAMERA = False
_C.HAS_GRIPPER = False

_ARMC = _C.ARM
_ARMC.CLASS = 'UrArm'
_ARMC.MOVEGROUP_NAME = 'manipulator'
# TODO: need to check this part
_ARMC.ARM_BASE_FRAME = 'base_link'
##
_ARMC.EE_FRAME = 'ee_link'
_ARMC.ROSTOPIC_JOINT_STATES = '/joint_states'
# TODO: need to check this part
_ARMC.ROSTOPIC_SET_JOINT = ''
##
_ARMC.IK_POSITION_TOLERANCE = 0.005
_ARMC.IK_ORIENTATION_TOLERANCE = 0.005

def get_cfg():
    return _C.clone()
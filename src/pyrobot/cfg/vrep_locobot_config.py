# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from yacs.config import CfgNode as CN

from pyrobot.cfg.config import get_cfg_defaults


_C = get_cfg_defaults()

# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a mobile base or not
_C.HAS_BASE = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a gripper or not
_C.HAS_GRIPPER = True
# whether the robot has a common shared class among all components
_C.HAS_COMMON = True


# Camera specific setting
_CAMERAC = _C.CAMERA
# CAMERA class name
_CAMERAC.CLASS = "LoCoBotCamera"

# Base specific settings
_BASEC = _C.BASE
# BASE class name
_BASEC.CLASS = "LoCoBotBase"


# Arm specific settings
_ARMC = _C.ARM
# Arm class name
_ARMC.CLASS = "LoCoBotArm"

# Gripper specific settings
_GRIPPERC = _C.GRIPPER
# Arm class name
_GRIPPERC.CLASS = "LoCoBotGripper"

_COMMONC = _C.COMMON
# Name of the common class variable that will be shared in Robot class
_COMMONC.NAME = "simulator"
# Class type to assign to 'simulator' variable
_COMMONC.CLASS = "VrepSim"

_C.COMMON.SIMULATOR = CN()

# Contains all of the simulator config
_SIMULATORC = _C.COMMON.SIMULATOR


def get_cfg():
    return _C.clone()

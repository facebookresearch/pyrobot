# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from pyrobot.cfg.config import get_cfg_defaults

_C = get_cfg_defaults()

_C.HAS_BASE = False
_C.HAS_CAMERA = False
_C.HAS_GRIPPER = True
_C.HAS_ARM = False

_GRIPPERC = _C.GRIPPER
_GRIPPERC.CLASS = "AllegroHand"
_GRIPPERC.MAX_JOINT_ERROR = 0.15
_GRIPPERC.WAIT_MIN_TIME = 1.5  # very important wait param
_GRIPPERC.JOINT_NAMES = [
    "joint_0.0",
    "joint_1.0",
    "joint_2.0",
    "joint_3.0",
    "joint_4.0",
    "joint_5.0",
    "joint_6.0",
    "joint_7.0",
    "joint_8.0",
    "joint_9.0",
    "joint_10.0",
    "joint_11.0",
    "joint_12.0",
    "joint_13.0",
    "joint_14.0",
    "joint_15.0",
]

_GRIPPERC.ROSPARAM_CONTROLLER = (
    "/allegroHand_0/controller"  # param name on which controller type is set
)
_GRIPPERC.ROSTOPIC_JOINT_STATES = (
    "/allegroHand_0/joint_states"  # subscribe to this topic for joint states
)
_GRIPPERC.ROSTOPIC_SET_JOINT = (
    "/allegroHand_0/joint_cmd"  # joint position command topic
)
_GRIPPERC.ROSTOPIC_SET_TORQUE = (
    "/allegroHand_0/torque_cmd"  # joint torque command topic
)
_GRIPPERC.ROSTOPIC_SET_PRIMITIVE = "/allegroHand_0/lib_cmd"

_GRIPPERC.PRIMITIVES = [
    "home",
    "ready",
    "pinch_it",
    "pinch_mt",
    "grasp_3",
    "grasp_4",
    "envelop",
    "gravcomp",
]


def get_cfg():
    return _C.clone()

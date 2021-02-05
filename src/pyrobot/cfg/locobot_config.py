# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from yacs.config import CfgNode as CN

from pyrobot.cfg.config import get_cfg_defaults

_C = get_cfg_defaults()

_ARMC = _C.ARM
# individual joint motor control command
_ARMC.ROSSERVICE_JOINT_COMMAND = "/joint_command"
# stop command for all motors
_ARMC.ROSTOPIC_STOP_EXECUTION = "/stop_execution"
# topic name to do torque control on LoCoBot
_ARMC.ROSTOPIC_TORQUE_COMMAND = "/torque_command"
# Arm Joint Names (Consistent with moveit group)
_ARMC.JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


_CAMERAC = _C.CAMERA
# CAMERA class name
_CAMERAC.CLASS = "LoCoBotCamera"
# minimum pan value allowed for the camera platform
_CAMERAC.MIN_PAN = -2.7
# maximum pan value allowed for the camera platform
_CAMERAC.MAX_PAN = 2.6
# minimum tilt value allowed for the camera platform
_CAMERAC.MIN_TILT = -1.4
# maximum tilt value allowed for the camera platform
_CAMERAC.MAX_TILT = 1.75
# reset value for the pan
_CAMERAC.RESET_PAN = 0.0
# reset value for the tilt
_CAMERAC.RESET_TILT = 0.0
# topic name of the camera info
_CAMERAC.ROSTOPIC_CAMERA_INFO_STREAM = "/camera/color/camera_info"
# topic name of the RGB images
_CAMERAC.ROSTOPIC_CAMERA_RGB_STREAM = "/camera/color/image_raw"
# topic name of the depth images
_CAMERAC.ROSTOPIC_CAMERA_DEPTH_STREAM = "/camera/aligned_depth_to_color/image_raw"
# topic name to set pan angle
_CAMERAC.ROSTOPIC_SET_PAN = "/pan/command"
# topic name to set tilt angle
_CAMERAC.ROSTOPIC_SET_TILT = "/tilt/command"
# Factor to scale depth image by to convert it into meters
_CAMERAC.DEPTH_MAP_FACTOR = 1000

_BASEC = _C.BASE
# BASE class name
_BASEC.CLASS = "LoCoBotBase"
# Type of base being used 'kobuki' or 'create'
_BASEC.BASE_TYPE = "kobuki"
# Type of contrller being used for postion control
# 'ilqr' or 'proportional' or 'movebase'
_BASEC.BASE_CONTROLLER = "ilqr"
# Type of planner being used for slam base path planning 'movebase'
_BASEC.BASE_PLANNER = "movebase"
# Rostopic on which the velocity commands to be published
_BASEC.ROSTOPIC_BASE_COMMAND = "/navigation_velocity_smoother/raw_cmd_vel"
# Rostopic on which the wheel-encoder-based odommetry is available
_BASEC.ROSTOPIC_ODOMETRY = "/odom"
# Rostopic on which base bumper sensor informations is available
_BASEC.ROSTOPIC_BUMPER = "/mobile_base/events/bumper"
# Rosotopic on which base cliff sensor information is available
_BASEC.ROSTOPIC_CLIFF = "/mobile_base/events/cliff"
# Rostopic on whihc the base wheeldrop sensor info is available
_BASEC.ROSTOPIC_WHEELDROP = "/mobile_base/events/wheel_drop"

# Rostopic on which movebase goal to be pusblished
_BASEC.ROSTOPIC_MOVE_BASE_GOAL = "/move_base_simple/goal"
# Rostopic on which the movebase execution status is available
_BASEC.ROSTOPIC_MOVE_BASE_STATUS = "/move_base/status"
# Rostopic on which the command to cancel the goal sent to movebase should be
_BASEC.ROSTOPIC_GOAL_CANCEL = "/move_base/cancel"
# world frame name
_BASEC.MAP_FRAME = "map"
# Rosaction topic for movebase
_BASEC.ROSTOPIC_BASE_ACTION_COMMAND = "move_base"
# Rate of control for ILQR
_BASEC.BASE_CONTROL_RATE = 10
# Maximum linear for velocity control and ILQR
_BASEC.MAX_ABS_FWD_SPEED = 0.2
# Maximum rotational velocity for velocity control and ILQR
_BASEC.MAX_ABS_TURN_SPEED = 0.5
# ROSTOPIC to send movebase (x,ym theta) planner request
_BASEC.PLAN_TOPIC = "/move_base/GlobalPlanner/make_plan"
# Index of the point to be tracked on the plan.
# (used by Proportional and ILQR trajectory tracking)
_BASEC.TRACKED_POINT = 8
# Linear treshold used by trajectory tracking with proportional and ILQR
_BASEC.TRESHOLD_LIN = 0.15
# Tolearance to be used by movebase planner while generating plans
_BASEC.PLAN_TOL = 0.1
# z minimum cut-off height for slam-based costmap computation
_BASEC.Z_MIN_TRESHOLD_OCC_MAP = 0.1
# z maximum cut-off height for slam-based costmap computation
_BASEC.Z_MAX_TRESHOLD_OCC_MAP = 0.8
# proportional control specific max linear velocity
_BASEC.MAX_ABS_FWD_SPEED_P_CONTROLLER = 0.5
# proportional control specific max angular velocity
_BASEC.MAX_ABS_TURN_SPEED_P_CONTROLLER = 1
# proportional control specific ignore translation treshold
_BASEC.TRANSLATION_TRESHOLD = 0.01
# GPMP control requires the goal acceptable error
_BASEC.GOAL_TOLERANCE = 0.1
# GPMP control requires the maximum allowable execution time in seconds
_BASEC.EXEC_TIME = 600
# GPMP control requires the GPMP action server name
_BASEC.GPMP_SERVER_NAME = "/gpmp_controller"
# GPMP control requires the turtlebot trajectory server name
_BASEC.TURTLEBOT_TRAJ_SERVER_NAME = "/turtle/base_controller/trajectory"


_BASEC.VSLAM = CN()
# topic name of the camera pose
_BASEC.VSLAM.ROSTOPIC_CAMERA_POSE = "/orb_slam2_rgbd/slam/camera_pose"
# topic name of the camera trajectory
_BASEC.VSLAM.ROSTOPIC_CAMERA_TRAJ = "/orb_slam2_rgbd/slam/camera_traj"
# reference link name of the visual SLAM system, the pose of this frame
# in the first time step will be used to define
# the origin/orientation of the world frame
_BASEC.VSLAM.VSLAM_BASE_FRAME = "base_link"
# RGB camera center frame name
_BASEC.VSLAM.RGB_CAMERA_CENTER_FRAME = "camera_color_optical_frame"
# minimum depth values to be considered as valid
_BASEC.VSLAM.DEPTH_MIN = 0.2
# maximum depth values to be considered as valid
_BASEC.VSLAM.DEPTH_MAX = 1.5
# configuration file name for ORB-SLAM2
_BASEC.VSLAM.CFG_FILENAME = "realsense_d435.yaml"
# sample every n pixels in depth images during
# reconstruction (to save computation time)
_BASEC.VSLAM.SUBSAMPLE_PIXS = 1
# publishing frequence of the occupancy map
_BASEC.VSLAM.OCCUPANCY_MAP_RATE = 0.5

_GRIPPERC = _C.GRIPPER
# GRIPPER class name
_GRIPPERC.CLASS = "LoCoBotGripper"
# topic name to open gripper
_GRIPPERC.ROSTOPIC_GRIPPER_OPEN = "/gripper/open"
# topic name to close gripper
_GRIPPERC.ROSTOPIC_GRIPPER_CLOSE = "/gripper/close"
# joint names of the gripper joints
_GRIPPERC.ROSTOPIC_GRIPPER_STATE = "/gripper/state"


def get_cfg(base_type="kobuki"):
    global _C
    if base_type not in ["kobuki", "create"]:
        raise ValueError("Unsupported base type: {:s}".format(base_type))
    if base_type == "create":
        import os

        dir_path = os.path.dirname(os.path.realpath(__file__))
        create_env_file = os.path.join(dir_path, "create_base.yaml")
        _C.merge_from_file(create_env_file)
    return _C.clone()

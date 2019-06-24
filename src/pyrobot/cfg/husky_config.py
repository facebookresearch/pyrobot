
from yacs.config import CfgNode as CN

from config import get_cfg_defaults

_C = get_cfg_defaults()

_BASEC = _C.BASE
# BASE class name
_BASEC.CLASS = 'HuskyBase'
# Type of base being used 'kobuki' or 'create'
_BASEC.BASE_TYPE = 'husky'
# Rostopic on which the velocity commands to be published
_BASEC.ROSTOPIC_BASE_COMMAND = '/husky_velocity_controller/cmd_vel'
# Rostopic on which the wheel-encoder-based odommetry is available
_BASEC.ROSTOPIC_ODOMETRY = '/odometry/filtered'
# Rostopic on which base bumper sensor informations is available
#_BASEC.ROSTOPIC_BUMPER = '/mobile_base/events/bumper'
# Rosotopic on which base cliff sensor information is available
#_BASEC.ROSTOPIC_CLIFF = '/mobile_base/events/cliff'
# Rostopic on whihc the base wheeldrop sensor info is available
#_BASEC.ROSTOPIC_WHEELDROP = '/mobile_base/events/wheel_drop'

# Rostopic on which movebase goal to be pusblished
_BASEC.ROSTOPIC_MOVE_BASE_GOAL = '/move_base_simple/goal'
# Rostopic on which the movebase execution status is available
_BASEC.ROSTOPIC_MOVE_BASE_STATUS = '/move_base/status'
# Rostopic on which the command to cancel the goal sent to movebase should be
_BASEC.ROSTOPIC_GOAL_CANCEL = '/move_base/cancel'
# world frame name
_BASEC.MAP_FRAME = '/map'
# Rosaction topic for movebase
_BASEC.ROSTOPIC_BASE_ACTION_COMMAND = 'move_base'
# Rate of control for ILQR
_BASEC.BASE_CONTROL_RATE = 10
# Maximum linear for velocity control and ILQR
_BASEC.MAX_ABS_FWD_SPEED = 5.0
# Maximum rotational velocity for velocity control and ILQR
_BASEC.MAX_ABS_TURN_SPEED = 2.0
# Type of planner being used for slam base path planning 'movebase'
_BASEC.BASE_PLANNER = 'movebase'
# ROSTOPIC to send movebase (x,ym theta) planner request
_BASEC.PLAN_TOPIC = '/move_base/NavfnROS/plan'
# Index of the point to be tracked on the plan.
# (used by Proportional and ILQR trajectory tracking)
_BASEC.TRACKED_POINT = 20
# Linear treshold used by trajectory tracking with proportional and ILQR
_BASEC.TRESHOLD_LIN = 0.15
# Tolearance to be used by movebase planner while generating plans
_BASEC.PLAN_TOL = 0.1
# z minimum cut-off height for slam-based costmap computation
_BASEC.Z_MIN_TRESHOLD_OCC_MAP = 0.1
# z maximum cut-off height for slam-based costmap computation
_BASEC.Z_MAX_TRESHOLD_OCC_MAP = 0.8
# proportional control specific max linear velocity
_BASEC.MAX_ABS_FWD_SPEED_P_CONTROLLER = 5.0
# proportional control specific max angular velocity
_BASEC.MAX_ABS_TURN_SPEED_P_CONTROLLER = 2.0

_BASEC.VSLAM = CN()
# topic name of the camera pose
_BASEC.VSLAM.ROSTOPIC_CAMERA_POSE = '/orb_slam2_rgbd/slam/camera_pose'
# topic name of the camera trajectory
_BASEC.VSLAM.ROSTOPIC_CAMERA_TRAJ = '/orb_slam2_rgbd/slam/camera_traj'
# reference link name of the visual SLAM system, the pose of this frame
# in the first time step will be used to define
# the origin/orientation of the world frame
_BASEC.VSLAM.VSLAM_BASE_FRAME = '/base_link'
# RGB camera center frame name
_BASEC.VSLAM.RGB_CAMERA_CENTER_FRAME = '/camera_color_optical_frame'
# minimum depth values to be considered as valid
_BASEC.VSLAM.DEPTH_MIN = 0.2
# maximum depth values to be considered as valid
_BASEC.VSLAM.DEPTH_MAX = 1.5
# configuration file name for ORB-SLAM2
_BASEC.VSLAM.CFG_FILENAME = 'realsense_d435.yaml'
# sample every n pixels in depth images during
# reconstruction (to save computation time)
_BASEC.VSLAM.SUBSAMPLE_PIXS = 1
# publishing frequence of the occupancy map
_BASEC.VSLAM.OCCUPANCY_MAP_RATE = 0.5

#
def get_cfg(base_type='kobuki'):
    global _C
    if base_type not in ['kobuki', 'create']:
        raise ValueError('Unsupported base type: {:s}'.format(base_type))
    if base_type == 'create':
        import os
        dir_path = os.path.dirname(os.path.realpath(__file__))
        create_env_file = os.path.join(dir_path, 'create_base.yaml')
        _C.merge_from_file(create_env_file)
    return _C.clone()

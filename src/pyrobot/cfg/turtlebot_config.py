
from yacs.config import CfgNode as CN

from config import get_cfg_defaults

_C = get_cfg_defaults()

_BASEC = _C.BASE
# BASE class name
_BASEC.CLASS = 'TurtlebotBase'
# Type of base being used 'kobuki' or 'create'
_BASEC.BASE_TYPE = 'kobuki'
# Type of contrller being used for postion control
# 'ilqr' or 'proportional' or 'movebase'
# Type of planner being used for slam base path planning 'movebase'
_BASEC.BASE_PLANNER = 'movebase'
# Rostopic on which the velocity commands to be published
_BASEC.ROSTOPIC_BASE_COMMAND = '/navigation_velocity_smoother/raw_cmd_vel'
# Rostopic on which the wheel-encoder-based odommetry is available
_BASEC.ROSTOPIC_ODOMETRY = '/odom'
# Rostopic on which base bumper sensor informations is available
_BASEC.ROSTOPIC_BUMPER = '/mobile_base/events/bumper'
# Rosotopic on which base cliff sensor information is available
_BASEC.ROSTOPIC_CLIFF = '/mobile_base/events/cliff'
# Rostopic on whihc the base wheeldrop sensor info is available
_BASEC.ROSTOPIC_WHEELDROP = '/mobile_base/events/wheel_drop'


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


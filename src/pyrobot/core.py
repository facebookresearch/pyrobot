# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import copy
import importlib
import os
import sys
import threading
import time
from abc import ABCMeta, abstractmethod

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import JointState, CameraInfo, Image

import pyrobot.utils.util as prutil

from pyrobot.utils.move_group_interface import MoveGroupInterface as MoveGroup
from pyrobot_bridge.srv import *

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError

import message_filters

import actionlib
from pyrobot_bridge.msg import (
    MoveitAction,
    MoveitGoal,
)
from actionlib_msgs.msg import GoalStatus



from hydra.experimental import initialize, compose
from hydra.utils import instantiate
from omegaconf import DictConfig
import logging
import os

def make_module(cfg):
    module =  instantiate(cfg.object, configs=cfg.conf)
    return module


def make_robot(robot_cfg, ns='', overrides=[], ros_launch_manager=None):

    # TODO: add check if the robot name passed is correct        
    

    if isinstance(robot_cfg, str):
        robot_cfg=compose("robot/" + robot_cfg + ".yaml", overrides)
    elif not isinstance(robot_cfg, DictConfig):
        raise ValueError('Expected either robot name or robot config object')
    ros_launch_manager.launch_cfg(robot_cfg.ros_launch, ns=ns)

    module_dict ={}
    for module_cfg in robot_cfg.modules:
        module_dict[module_cfg.name] = make_module(module_cfg)

    return module_dict

def make_sensor(self, sensor_cfg, ns='', overrides=[], ros_launch_manager=None):
        
    # TODO: add check if the sensor name passed is correct        
    

    if isinstance(sensor_cfg, str):
        sensor_cfg=compose("sensor/" + sensor_cfg + ".yaml", overrides)
    elif not isinstance(sensor_cfg, DictConfig):
        raise ValueError('Expected either robot name or robot config object')
    ros_launch_manager.launch_cfg(sensor_cfg.ros_launch, ns=ns)

    module_dict ={}
    for module_cfg in sensor_cfg.modules:
        module_dict[module_cfg.name] = make_module(module_cfg)
    return module_dict

class World(object):
    """
    TODO: Add more detailed info here.
    Root class for all problem

    """

    def __init__(self, configs_path=None, config_name=None, overrides=[]):

        # overrides: https://hydra.cc/docs/next/experimental/compose_api/#internaldocs-banner
        # TODO: see if ros can be made optional
        self.ros_launch_manager = RosLaunchManager()

        if self.ros_launch_manager.get_window('roscore') is None:
            self.ros_launch_manager.create_and_cmd(name='roscore', cmd='roscore', ns='')
            rospy.sleep(1)


        try:
            rospy.init_node("pyrobot", anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn("ROS node [pyrobot] has already been initialized")

        if configs_path is None:
            configs_path = 'hydra_config'
        try:
            initialize(configs_path)
        except AssertionError as error:
            logging.warning(error)

        
        self.robots = {}
        self.sensors = {}
        self.algorithms = {}
        self.objects = {}
        self.simulator = {}
        # Any sensor streams and commands are stored here
        self.measurements = {}

        if config_name is not None:

            world_config = compose(config_name, overrides)

            if world_config.environment.robots is not None:
                for robot in world_config.environment.robots:
                    self.add_robot(robot.robot, ns= robot.ns, overrides=robot.overrides)

            if world_config.environment.sensors is not None:
                for sensor in world_config.environment.sensors:
                    self.add_sensor(sensor.sensor, ns= sensor.ns, overrides= sensor.overrides)                

            # TODO: Add, how to deal with objects, and obstacle module

    def add_robot(self, robot_cfg, ns='', overrides=[]):
        self.robots[robot_cfg.name] = make_robot(robot_cfg, ns, overrides, self.ros_launch_manager)


    def add_sensor(self, sensor_cfg, ns='', overrides=[]):
        self.sensor[sensor_cfg.name] = make_sensor(sensor_cfg, ns, overrides,self.ros_launch_manager)
    
    def add_algorithm(self, sensor_cfg, ns='', overrides=[]):
        pass





import libtmux
import os
import os.path as osp
class  RosLaunchManager(object):
    """
    TODO: Add more details.
    """
    def __init__(self, world_ns='pyrobot', scope='extend'):

        # scope = 'overwrite' or 'extend'
        
        self.server =  libtmux.Server()
        self.world_ns = world_ns
        self.scope = scope
        if scope == 'overwrite':
            self.kill_session(self.world_ns)

        if self.server.has_session(self.world_ns):
            self.session = self.server.find_where({ "session_name": self.world_ns})
        else:
            self.session = self.server.new_session(self.world_ns)

        #print("Available sessions:", self.server.list_sessions() )
        # maps from any node names to the window names.
        self.process_dict = {} 
        self.window_names = []
        self.scope = scope
        
    def set_env_ros(self, pane):
        pane.send_keys('source /opt/ros/melodic/setup.bash')
        pane.send_keys('source ~/low_cost_ws/devel/setup.bash')
        pane.send_keys('source ~/pyenv_pyrobot_python2/bin/activate')
        
    def set_ros_python(self, pane):
        pane.send_keys('source /opt/ros/melodic/setup.bash')
        pane.send_keys('source ~/low_cost_ws/devel/setup.bash')
        pane.send_keys('source ~/pyrobot_catkin_ws/devel/setup.bash')
        pane.send_keys('source ~/pyenv_pyrobot_python3/bin/activate')
        # pane.send_keys('load_pyrobot_env')
        
    def kill_session(self, sess_name):
        if self.server.has_session(sess_name):
            self.server.kill_session(sess_name)

    def get_window(self, window_name):
        windows = self.session.windows
        if windows is None:
            return None
        for window in windows:
            if window.name == window_name:
                return window
        return None

    def kill_window(self, window_name):
        window = self.get_window(window_name)
        if window is not None:
            window.kill_window()

    def create_and_cmd(self, name, cmd, ns=''):
        window = self.get_window(name)
        if window is None:
            self.window_names.append(name)
            window = self.session.new_window(name)
        else:
            if self.scope == 'overwrite':
                print("Overwriting the window: {}".format(name))
                self.kill_window(name)
                window = self.session.new_window(name)
            else:
                pane = window.panes[0]
                pane.send_keys('C-c', enter=False, suppress_history=False)
                print("Window already exists. Killing past commands and running new.")
                #print("Cannot overwite. A window with name {} already exits".format(name))
                #return

        pane = window.panes[0]
        self.set_env_ros(pane)
        # TODO: add namespace bash variable here!
        if ns !='':
            pane.send_keys('export ROS_NAMESPACE=/{}'.format(ns))
        pane.send_keys(cmd)

    def launch_cfg(self, ros_cfg, ns = ''):
        mode = ros_cfg.mode
        wait_time = ros_cfg.wait

        if isinstance(ros_cfg[mode], str):
            cmd = ros_cfg[mode]
            if cmd is not  None:
                name = osp.join(ns, ros_cfg.name)
                self.create_and_cmd(name, cmd + ' ns:={}'.format(ns),ns)
        else: 
            for key in ros_cfg[mode]:
                cmd = ros_cfg[mode][key]
                if cmd is not  None:
                    name = osp.join(ns, ros_cfg.name)
                    self.create_and_cmd(name, cmd + ' ns:={}'.format(ns), ns)

        rospy.sleep(wait_time) #seconds

    # TODO: fixt this
    # def __del__(self):
    #     if self.scope == 'overwrite':
    #         self.kill_session(self.world_ns)
    #     else:
    #         for window_name in self.window_names:
    #             self.kill_window(window_name)

# from abc import ABC, abstractmethod 
# class Algorithm(ABC):
#     """Algorithm base class on which everything else is bulit"""
#     def __init__( self, cfg, 
#                         ros_launch_manager = None, 
#                         robots={}, 
#                         sensors={}, 
#                         algorithms={}, 
#                         world={}
#                 ):

#         self.cfg = cfg
#         if not self.check_cfg():
#             print("Invalid config encoutered")
#             return
#         ros_launch_manager.launch(world_ns=None, robot_ns=None, algo_ns=self.cfg.ns)

#     @abstractmethod
#     def check_cfg(self):
#         pass




# class Kinematics(Algorithm):
#     """docstring for Kinematics"""
#     def __init__(self, cfg):
        
#         super(Algorithm, self).__init__(
#                         cfg, 
#                         ros_launch_manager = None, 
#                         robots={}, 
#                         sensors={}, 
#                         algorithms={}, 
#                         world={})
#         self.arg = arg
#         
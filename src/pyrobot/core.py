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
    ros_launch_cmds = None
    return module, None

def launch_ros_backend(roslaunch_cfg):
    # https://www.geeksforgeeks.org/create-xml-documents-using-python/
    pass


# class Piper(object):
#     """
#     The main interface object with all the sensor streams and commands
#     """

#     def __init__(self, configs_path=None):

#         self.data = {}
#         self.commands = {}


class World(object):
    """
    TODO: Add more detailed info here.
    Root class for all problem

    """

    def __init__(self, configs_path=None, config_name=None, node_ns=''):


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

        self.modules ={}
        self.robots = {}
        self.sensors = {}
        self.algorithms = {}
        self.objects = {}
        self.simulator = {}
        self.roslaunch_cfg = [] # TODO: May have to change this to dict

        # Any sensor streams and commands are stored here
        self.measurements = {}

        if config_name is not None:

            world_config = compose(config_name)

            if world_config.environment.robots is not None:
                for robot in world_config.environment.robots:
                    print("youyoyyo")
                    self.add_robot(robot.robot, robot.overrides)

            if world_config.environment.sensors is not None:
                for sensor in world_config.environment.sensors:
                    self.add_sensor(sensor.sensor, sensor.overrides)                

            # TODO: Add, how to deal with objects, and obstacle module

    def add_robot(self, robot_cfg, overrides=[]):


        # TODO: add check if the robot name passed is correct        
        # TODO: Check the rosparam server if the robot already exits!!!
        if isinstance(robot_cfg, str):
            robot_cfg=compose("robot/" + robot_cfg + ".yaml")
        elif not isinstance(robot_cfg, DictConfig):
            raise ValueError('Expected either robot name or robot config object')

        module_dict ={}
        for module_cfg in robot_cfg.modules:
            module_dict[module_cfg.name], roslaunch_cfg = make_module(module_cfg)
            self.roslaunch_cfg.append(roslaunch_cfg)
        self.robots[robot_cfg.name] = module_dict


    def add_sensor(self, sensor_cfg, overrides=[]):
        
        # TODO: add check if the sensor name passed is correct        
        # TODO: Check the rosparam server if the sensor already exits!!!
        if isinstance(sensor_cfg, str):
            sensor_cfg=compose("sensor/" + sensor_cfg + ".yaml")
        elif not isinstance(sensor_cfg, DictConfig):
            raise ValueError('Expected either robot name or robot config object')

        module_dict ={}
        for module_cfg in sensor_cfg.modules:
            module_dict[module_cfg.name], roslaunch_cfg = make_module(module_cfg)
            self.roslaunch_cfg.append(roslaunch_cfg)
        self.sensor[sensor_cfg.name] = module_dict
    
    def add_algorithm(self, sensor_cfg, overrides=[]):
        pass

    def launch_ros(self):

        launch_ros_backend(self.roslaunch_cfg)






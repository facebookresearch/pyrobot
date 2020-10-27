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

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

from cv_bridge import CvBridge, CvBridgeError

import message_filters

import actionlib

from actionlib_msgs.msg import GoalStatus


from hydra.experimental import initialize, compose
from hydra.utils import instantiate
from omegaconf import DictConfig
import logging
import os


import libtmux
import os
import os.path as osp


def make_module(cfg):
    print(cfg)
    module = instantiate(cfg.object, configs=cfg.conf)
    return module


def make_robot(robot_cfg, ns="", overrides=[], ros_launch_manager=None):

    if isinstance(robot_cfg, str):
        robot_cfg = compose("robot/" + robot_cfg + ".yaml", overrides)
    elif not isinstance(robot_cfg, DictConfig):
        raise ValueError("Expected either robot name or robot config object")
    ros_launch_manager.launch_cfg(robot_cfg.ros_launch, ns=ns)

    module_dict = {}
    for module_cfg in robot_cfg.modules:
        module_dict[module_cfg.name] = make_module(module_cfg)

    return module_dict


def make_sensor(sensor_cfg, ns="", overrides=[], ros_launch_manager=None):

    # TODO: add check if the sensor name passed is correct

    if isinstance(sensor_cfg, str):
        sensor_cfg = compose("sensor/" + sensor_cfg + ".yaml", overrides)
    elif not isinstance(sensor_cfg, DictConfig):
        raise ValueError("Expected either robot name or robot config object")
    ros_launch_manager.launch_cfg(sensor_cfg.ros_launch, ns=ns)

    module_dict = {}
    for module_cfg in sensor_cfg.modules:
        module_dict[module_cfg.name] = make_module(module_cfg)
    return module_dict


def make_algorithm(algorithm_cfg, world, ns="", overrides=[], ros_launch_manager=None):

    # TODO: add check if the algorithm name passed is correct
    if isinstance(algorithm_cfg, str):
        algorithm_cfg = compose("algorithm/" + algorithm_cfg + ".yaml", overrides)
    elif not isinstance(algorithm_cfg, DictConfig):
        raise ValueError("Expected either algorithm name or algorithm config object")

    robots = {}
    if "robot_labels" in algorithm_cfg.keys() and algorithm_cfg.robot_labels:
        for robot_label in algorithm_cfg.robot_labels:
            robots[robot_label] = world.robots[robot_label]

    # TODO: add sensors
    sensors = {}

    dependencies = {}
    if "dependencies" in algorithm_cfg.keys() and algorithm_cfg.dependencies:
        for dependency_cfg in algorithm_cfg.dependencies:
            if dependency_cfg["algorithm"] in world._algorithms_pool.keys():
                dependency = world._algorithms_pool[dependency_cfg["algorithm"]]
            else:
                dependency = make_algorithm(
                    dependency_cfg["algorithm"], world, ns, overrides, ros_launch_manager
                )
                world._algorithms_pool[dependency_cfg["algorithm"]] = dependency

            dependencies[dependency.get_class_name()] = dependency

    if "ros_launch" in algorithm_cfg.keys() and algorithm_cfg.ros_launch:
        if not ros_launch_manager:
            ros_launch_manager = world.ros_launch_manager
        ros_launch_manager.launch_cfg(algorithm_cfg.ros_launch, ns=ns)

    if "conf" not in algorithm_cfg.keys():
        conf = {}
    else:
        conf = algorithm_cfg.conf

    algorithm = instantiate(
        algorithm_cfg.algorithm,
        configs=conf,
        world=world,
        ros_launch_manager=ros_launch_manager,
        robots=robots,
        sensors=sensors,
        algorithms=dependencies,
    )
    return algorithm


class World(object):
    """
    Root class for four types or modules: robots, sensors, algorithms, and objects (obstacles), each being a dictionary of robot/sensors/algorithms/object instances.
    All of these instances can be queried by self.robots/sensors/algorithms/objectsp["label"]
    """

    def __init__(self, config_name=None, configs_path=None, overrides=[]):
        """
        Constructor for World object.

        :param configs_name: file name for the config file correspond to the world object. Type: str
        :param configs_path: folder path for the config file correspond to the world object. If None, default to be `{$pyrobot-root}/src/pyrobot/hydra_config`. Type: str
        :param overrides: a list of override commands. Details described in: https://hydra.cc/docs/next/experimental/compose_api/#internaldocs-banner. Type: list[str].
        """

        # overrides: https://hydra.cc/docs/next/experimental/compose_api/#internaldocs-banner
        # TODO: see if ros can be made optional
        self.ros_launch_manager = RosLaunchManager()

        if self.ros_launch_manager.get_window("roscore") is None:
            self.ros_launch_manager.create_and_cmd(name="roscore", cmd="roscore", ns="")
            rospy.sleep(1)

        try:
            rospy.init_node("pyrobot", anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn("ROS node [pyrobot] has already been initialized")

        if configs_path is None:
            configs_path = "hydra_config"
        try:
            initialize(configs_path)
        except AssertionError as error:
            logging.warning(error)

        self.robots = {}
        self.sensors = {}
        # public accessible algorithms
        self.algorithms = {}
        # pool of all the available algorithms in the backend
        self._algorithms_pool = {}

        self.objects = {}
        self.simulator = {}
        # Any sensor streams and commands are stored here
        self.measurements = {}

        if config_name is not None:

            world_config = compose(config_name, overrides)

            if world_config.environment.robots is not None:
                for robot in world_config.environment.robots:
                    self.add_robot(
                        robot.robot, robot.label, ns=robot.ns, overrides=robot.overrides
                    )

            if world_config.environment.sensors is not None:
                for sensor in world_config.environment.sensors:
                    self.add_sensor(
                        sensor.sensor,
                        sensor.label,
                        ns=sensor.ns,
                        overrides=sensor.overrides,
                    )

            if world_config.environment.algorithms is not None:
                for algorithm in world_config.environment.algorithms:
                    self.add_algorithm(
                        algorithm.algorithm,
                        algorithm.label,
                        ns=algorithm.ns,
                        overrides=algorithm.overrides,
                    )

            # TODO: Add, how to deal with objects, and obstacle module

    def add_robot(self, robot_cfg, label, ns="", overrides=[]):
        self.robots[label] = make_robot(
            robot_cfg, ns, overrides, self.ros_launch_manager
        )

    def add_sensor(self, sensor_cfg, label, ns="", overrides=[]):
        self.sensors[label] = make_sensor(
            sensor_cfg, ns, overrides, self.ros_launch_manager
        )

    def add_algorithm(self, algorithm_cfg, label, ns="", overrides=[]):
        if label in self._algorithms_pool.keys():
            self.algorithms[label] = self._algorithms_pool[label]
        else:
            self.algorithms[label] = make_algorithm(
                algorithm_cfg, self, ns, overrides, self.ros_launch_manager
            )


class RosLaunchManager(object):
    """
    TODO: Add more details.
    """

    def __init__(self, world_ns="pyrobot", scope="extend"):

        # scope = 'overwrite' or 'extend'

        self.server = libtmux.Server()
        self.world_ns = world_ns
        self.scope = scope
        if scope == "overwrite":
            self.kill_session(self.world_ns)

        if self.server.has_session(self.world_ns):
            self.session = self.server.find_where({"session_name": self.world_ns})
        else:
            self.session = self.server.new_session(self.world_ns)

        # print("Available sessions:", self.server.list_sessions() )
        # maps from any node names to the window names.
        self.process_dict = {}
        self.window_names = []
        self.scope = scope

    def set_env_ros(self, pane):
        pane.send_keys("source /opt/ros/melodic/setup.bash")
        pane.send_keys("source ~/low_cost_ws/devel/setup.bash")
        pane.send_keys("source ~/pyenv_pyrobot_python2/bin/activate")

    def set_ros_python(self, pane):
        pane.send_keys("source /opt/ros/melodic/setup.bash")
        pane.send_keys("source ~/low_cost_ws/devel/setup.bash")
        pane.send_keys("source ~/pyrobot_catkin_ws/devel/setup.bash")
        pane.send_keys("source ~/pyenv_pyrobot_python3/bin/activate")
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

    def create_and_cmd(self, name, cmd, ns=""):
        window = self.get_window(name)
        if window is None:
            self.window_names.append(name)
            window = self.session.new_window(name)
        else:
            if self.scope == "overwrite":
                print("Overwriting the window: {}".format(name))
                self.kill_window(name)
                window = self.session.new_window(name)
            else:
                pane = window.panes[0]
                pane.send_keys("C-c", enter=False, suppress_history=False)
                print("Window already exists. Killing past commands and running new.")
                # print("Cannot overwite. A window with name {} already exits".format(name))
                # return

        pane = window.panes[0]
        self.set_env_ros(pane)
        # TODO: add namespace bash variable here!
        if ns != "":
            pane.send_keys("export ROS_NAMESPACE=/{}".format(ns))
        pane.send_keys(cmd)

    def launch_cfg(self, ros_cfg, ns=""):
        mode = ros_cfg.mode
        wait_time = ros_cfg.wait

        if isinstance(ros_cfg[mode], str):
            cmd = ros_cfg[mode]
            if cmd is not None:
                name = osp.join(ns, ros_cfg.name)
                self.create_and_cmd(name, cmd + " ns:={}".format(ns), ns)
        else:
            for key in ros_cfg[mode]:
                cmd = ros_cfg[mode][key]
                if cmd is not None:
                    name = osp.join(ns, ros_cfg.name)
                    self.create_and_cmd(name, cmd + " ns:={}".format(ns), ns)

        rospy.sleep(wait_time)  # seconds

    def __del__(self):
        if self.scope == "overwrite":
            self.kill_session(self.world_ns)
        else:
            for window_name in self.window_names:
                self.kill_window(window_name)

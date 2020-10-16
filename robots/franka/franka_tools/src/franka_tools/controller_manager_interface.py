#!/usr/bin/env python

# /***************************************************************************

# 
# @package: franka_tools
# @metapackage: franka_ros_interface
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2020, Saif Sidhik
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

import rospy
import numpy as np
from copy import deepcopy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import *
import socket
from franka_core_msgs.msg import JointControllerStates

from franka_tools import ControllerParamConfigClient

from controller_manager_msgs.utils import ( ControllerLister, 
                                get_rosparam_controller_names)

def _resolve_controllers_ns(cm_ns):
    """
    Resolve the namespace containing controller configurations from that of
    the controller manager.
    Controllers are assumed to live one level above the controller
    manager, e.g.
        >>> _resolve_controller_ns('/path/to/controller_manager')
        '/path/to'
    In the particular case in which the controller manager is not
    namespaced, the controller is assumed to live in the root namespace
        >>> _resolve_controller_ns('/')
        '/'
        >>> _resolve_controller_ns('')
        '/'
    :param cm_ns Controller manager namespace (can be an empty string)
    :type cm_ns str
    :return: Controllers namespace
    :rtype: str
    """
    ns = cm_ns.rsplit('/', 1)[0]
    if not ns:
        ns += '/'
    return ns


def _append_ns(in_ns, suffix):
    """
    Append a sub-namespace (suffix) to the input namespace
    :param in_ns Input namespace
    :type in_ns str
    :return: Suffix namespace
    :rtype: str
    """
    ns = in_ns
    if ns[-1] != '/':
        ns += '/'
    ns += suffix
    return ns


def _rosparam_controller_type(ctrls_ns, ctrl_name):
    """
    Get a controller's type from its ROS parameter server configuration
    :param ctrls_ns Namespace where controllers should be located
    :type ctrls_ns str
    :param ctrl_name Controller name
    :type ctrl_name str
    :return: Controller type
    :rtype: str
    """
    type_param = _append_ns(ctrls_ns, ctrl_name) + '/type'
    return rospy.get_param(type_param)

def _get_controller_name_from_rosparam_server(rosparam_name):
    try:
        cname = rospy.get_param(rosparam_name)
    except KeyError:
        rospy.loginfo("FrankaControllerManagerInterface: cannot detect controller name"
                        " under param {}".format(rosparam_name))
        cname = ''
    except (socket.error, socket.gaierror):
        rospy.logerr("Failed to connect to the ROS parameter server!\n"
            "Please check to make sure your ROS networking is "
            "properly configured:\n")
        sys.exit()

    return cname

class ControllerStateInfo:
    """
        Class for creating a storage object for controller state published by the induvidual franka_ros_controllers.
    """
    def __init__(self, controller_state_msg):
        self.controller_name = controller_state_msg.controller_name
        self.p = np.asarray([j.p for j in controller_state_msg.joint_controller_states])
        self.d = np.asarray([j.d for j in controller_state_msg.joint_controller_states])
        self.i = np.asarray([j.i for j in controller_state_msg.joint_controller_states])
        self.process_value = np.asarray([j.process_value for j in controller_state_msg.joint_controller_states])
        self.set_point = np.asarray([j.set_point for j in controller_state_msg.joint_controller_states])
        self.process_value_dot = np.asarray([j.process_value_dot for j in controller_state_msg.joint_controller_states])
        self.error = np.asarray([j.error for j in controller_state_msg.joint_controller_states])
        self.time_step = np.asarray([j.time_step for j in controller_state_msg.joint_controller_states])
        self.i_clamp = np.asarray([j.i_clamp for j in controller_state_msg.joint_controller_states])
        self.command = np.asarray([j.command for j in controller_state_msg.joint_controller_states])

class FrankaControllerManagerInterface(object):
    """
    
    :type synchronous_pub: bool
    :param synchronous_pub: designates the JointCommand Publisher
        as Synchronous if True and Asynchronous if False.

        Synchronous Publishing means that all joint_commands publishing to
        the robot's joints will block until the message has been serialized
        into a buffer and that buffer has been written to the transport
        of every current Subscriber. This yields predicable and consistent
        timing of messages being delivered from this Publisher. However,
        when using this mode, it is possible for a blocking Subscriber to
        prevent the joint_command functions from exiting. Unless you need exact
        JointCommand timing, default to Asynchronous Publishing (False).

    :param ns: base namespace of interface ('frank_ros_interface'/'panda_simulator')
    :type ns: str

    :param sim: Flag specifying whether the robot is in simulation or not 
        (can be obtained from :py:class:`franka_interface.RobotParams` instance)
    :type sim: bool
    """

    def __init__(self, ns="franka_ros_interface", synchronous_pub = False, sim = False):

        self._ns = ns if ns[0] == '/' else '/' + ns
        self._prefix = "/controller_manager"
        self._cm_ns = self._prefix
        self._service_names = ["list_controllers",
                             "unload_controller",
                             "load_controller",
                             "switch_controller"]


        load_srv_name = self._cm_ns + "/load_controller"
        self._load_srv = rospy.ServiceProxy(load_srv_name,
                                                LoadController,
                                                persistent=True)
        unload_srv_name = self._cm_ns + "/unload_controller"
        self._unload_srv = rospy.ServiceProxy(unload_srv_name,
                                                  UnloadController,
                                                  persistent=True)
        switch_srv_name = self._cm_ns + "/switch_controller"
        self._switch_srv = rospy.ServiceProxy(switch_srv_name,
                                                  SwitchController,
                                                  persistent=True)

        list_srv_name = self._cm_ns + "/list_controllers"
        self._list_srv = rospy.ServiceProxy(list_srv_name,
                                                  ListControllers,
                                                  persistent=True)

        list_types_srv_name = self._cm_ns + "/list_controller_types"
        self._list_types_srv = rospy.ServiceProxy(list_types_srv_name,
                                                  ListControllerTypes,
                                                  persistent=True)

        self._in_sim = sim

        self._controller_lister = ControllerLister(self._cm_ns)

        self._controller_names_from_rosparam = {
            'joint_position_controller': _get_controller_name_from_rosparam_server('/controllers_config/position_controller'),
            'joint_velocity_controller': _get_controller_name_from_rosparam_server('/controllers_config/velocity_controller'),
            'joint_torque_controller': _get_controller_name_from_rosparam_server('/controllers_config/torque_controller'),
            'joint_trajectory_controller': _get_controller_name_from_rosparam_server('/controllers_config/trajectory_controller'),
            'effort_joint_position_controller': _get_controller_name_from_rosparam_server('/controllers_config/impedance_controller'),
            'default_controller': _get_controller_name_from_rosparam_server('/controllers_config/default_controller'),
        }


        if self._in_sim:
            self._non_motion_controllers = [self._ns[1:] + '/custom_franka_state_controller', self._ns[1:] + '/panda_gripper_controller', self._ns[1:] + '/effort_joint_gravity_controller', self._ns[1:] + '/joint_state_controller']
        else:
            self._non_motion_controllers = [self._ns[1:] + '/custom_franka_state_controller',self._ns[1:] + '/franka_state_controller']

        self._assert_one_active_controller()

        self._state_subscriber = rospy.Subscriber("%s/motion_controller/arm/joint_controller_states" %(self._ns),
                                                    JointControllerStates, self._on_controller_state, queue_size = 1,
                                                    tcp_nodelay = True)

        self._param_config_clients = {}
        self._dont_start_config_client = False


        rospy.on_shutdown(self._clean_shutdown)

    def _clean_shutdown(self):

        if self._state_subscriber:
            self._state_subscriber.unregister()


    def _on_controller_state(self, msg):
        self._controller_state = deepcopy(ControllerStateInfo(msg))

        self._assert_one_active_controller()

        if self._current_controller and self._current_controller not in self._param_config_clients:
            self._param_config_clients[self._current_controller] = ControllerParamConfigClient(self._current_controller)
            self._param_config_clients[self._current_controller].start()


    def _assert_one_active_controller(self):
        ctrlr_list = self.list_active_controllers(only_motion_controllers=True)
        assert len(ctrlr_list) <= 1, "FrankaControllerManagerInterface: More than one motion controller active!"
        self._current_controller = ctrlr_list[0].name if len(ctrlr_list) > 0 else None


    def load_controller(self, name):
        """
        Loads the specified controller

        :type name: str
        :param name: name of the controller to be loaded
        """
        self._load_srv.call(LoadControllerRequest(name=name))

    def unload_controller(self, name):
        """
        Unloads the specified controller

        :type name: str
        :param name: name of the controller to be unloaded
        """
        self._unload_srv.call(UnloadControllerRequest(name=name))

    def start_controller(self, name):
        """
        Starts the specified controller

        :type name: str
        :param name: name of the controller to be started
        """
        assert len(self.list_active_controllers(only_motion_controllers=True)) == 0, "FrankaControllerManagerInterface: One motion controller already active: %s. Stop this controller before activating another!"%self._current_controller

        strict = SwitchControllerRequest.STRICT
        req = SwitchControllerRequest(start_controllers=[name],
                                      stop_controllers=[],
                                      strictness=strict)
        rospy.logdebug("FrankaControllerManagerInterface: Starting controller: %s"%name)
        self._switch_srv.call(req)

        self._assert_one_active_controller()

    def get_controller_state(self):
        """
            Get the status of the current controller, including set points, computed 
            command, controller gains etc. See the ControllerStateInfo class (above) 
            parameters for more info.
        """
        return deepcopy(self._controller_state)

    def stop_controller(self, name):
        """
        Stops the specified controller

        :type name: str
        :param name: name of the controller to be stopped
        """
        strict = SwitchControllerRequest.STRICT
        req = SwitchControllerRequest(start_controllers=[],
                                      stop_controllers=[name],
                                      strictness=strict)
        rospy.logdebug("FrankaControllerManagerInterface: Stopping controller: %s"%name)
        self._switch_srv.call(req)


    def list_loaded_controllers(self):
        """
        :return: List of controller types associated to a controller manager
            namespace. Contains all loaded controllers, as returned by
            the `list_controller_types` service, plus uninitialized controllers with
            configurations loaded in the parameter server.
        :rtype: [str]
        """
        req = ListControllersRequest()

        return self._list_srv.call(req)
    
    def list_controller_types(self):
        """
        :return: List of controller types associated to a controller manager
            namespace. Contains both stopped/running/loaded controllers, as returned by
            the `list_controller_types` service, plus uninitialized controllers with
            configurations loaded in the parameter server.
        :rtype: [str]
        """
        req = ListControllerTypesRequest()

        return self._list_types_srv.call(req)

    def list_controllers(self):
        """
        :return: List of controllers associated to a controller manager
            namespace. Contains both stopped/running controllers, as returned by
            the `list_controllers` service, plus uninitialized controllers with
            configurations loaded in the parameter server.
        :rtype: [ControllerState obj]
        """
        if not self._cm_ns:
            return []

        # Add loaded controllers first
        controllers = self._controller_lister()

        # Append potential controller configs found in the parameter server
        all_ctrls_ns = _resolve_controllers_ns(self._cm_ns)
        for name in get_rosparam_controller_names(all_ctrls_ns):

            add_ctrl = not any(name == ctrl.name for ctrl in controllers)
            if add_ctrl:
                type_str = _rosparam_controller_type(all_ctrls_ns, name)
                uninit_ctrl = ControllerState(name=name,
                                              type=type_str,
                                              state='uninitialized')
                controllers.append(uninit_ctrl)
        for c in controllers:
            if c.name[0] == '/':
                c.name = c.name[1:]
        return controllers

    def controller_dict(self):
        """
        Get all controllers as dict

        :return: name of the controller to be stopped
        :rtype: dict {'controller_name': ControllerState}
        """
        controllers = self.list_controllers()

        controller_dict = {}
        for c in controllers:
            controller_dict[c.name] = c

        return controller_dict

    def set_motion_controller(self, controller_name):
        """
        Set the specified controller as the (only) motion controller
    
        :return: name of currently active controller (can be used to switch back to this later)
        :rtype: str
        :type controller_name: str
        :param controller_name: name of controller to start
        """

        if controller_name.strip() == '':
            rospy.logdebug("FrankaControllerManagerInterface: Empty controller name in controller switch request. Ignoring.")
            return self._current_controller

        if controller_name[0] == '/':
            controller_name = controller_name[1:]
        
        curr_ctrlr = self._current_controller
        switch_ctrl = (curr_ctrlr != controller_name)

        if switch_ctrl:
            active_controllers = self.list_active_controllers(only_motion_controllers = True)
            for ctrlr in active_controllers:
                self.stop_controller(ctrlr.name)
                rospy.sleep(0.5)

            if not self.is_loaded(controller_name):
                self.load_controller(controller_name)

            self.start_controller(controller_name)
        else:
            rospy.logdebug("FrankaControllerManagerInterface: Controller '{0}' already active. Not switching.".format(controller_name))

        return curr_ctrlr


    def is_running(self, controller_name):
        """
        Check if the given controller is running.

        :type controller_name: str
        :param controller_name: name of controller whose status is to be checked
        :return: True if controller is running, False otherwise
        :rtype: bool
        """
        controllers = self.controller_dict()

        ctrl_state = controllers.get(controller_name,None)

        return ctrl_state is not None and ctrl_state.state=="running"

    def is_loaded(self, controller_name):
        """
        Check if the given controller is loaded.

        :type controller_name: str
        :param controller_name: name of controller whose status is to be checked
        :return: True if controller is loaded, False otherwise
        :rtype: bool
        """
        controllers = self.controller_dict()

        ctrl_state = controllers.get(controller_name,None)

        return ctrl_state is not None and ctrl_state.state!="uninitialized"

    def list_motion_controllers(self):
        """
        :return: List of motion controllers associated to a controller manager
            namespace. Contains both stopped/running controllers, as returned by
            the `list_controllers` service, plus uninitialized controllers with
            configurations loaded in the parameter server.
        :rtype: [ControllerState obj]
        """
        motion_controllers = []
        for controller in self.list_controllers():
            if not controller.name in self._non_motion_controllers:
                motion_controllers.append(controller)

        return motion_controllers


    def list_active_controllers(self, only_motion_controllers = False):
        """
        :return: List of  active controllers associated to a controller manager
            namespace. Contains both stopped/running controllers, as returned by
            the `list_controllers` service, plus uninitialized controllers with
            configurations loaded in the parameter server.
        :rtype: [ControllerState obj]
        
        :param only_motion_controller: if True, only motion controllers are returned
        :type only_motion_controller: bool

        """
        if only_motion_controllers:
            controllers = self.list_motion_controllers()
        else:
            controllers = self.list_controllers()
        
        active_controllers = []
        for controller in controllers:
            if controller.state == "running":
                active_controllers.append(controller)

        return active_controllers

    def list_active_controller_names(self, only_motion_controllers = False):
        """
        :return: List of names active controllers associated to a controller manager
            namespace. 
        :rtype: [str]
        
        :param only_motion_controller: if True, only motion controllers are returned
        :type only_motion_controller: bool

        """
        return [c.name for c in self.list_active_controllers(only_motion_controllers = only_motion_controllers)]

    def get_controller_config_client(self, controller_name):
        """
        :return: The parameter configuration client object associated with the specified
            controller
        :rtype: ControllerParamConfigClient obj (if None, returns False)
        
        :param controller_name: name of controller whose config client is required
        :type controller_name: str

        """
        if controller_name in self._param_config_clients:
            return self._param_config_clients[controller_name]
        else:
            rospy.logwarn("FrankaControllerManagerInterface: No parameter configuration client available for controller {}".format(controller_name))
            return False

    def get_current_controller_config_client(self):
        """
        :return: The parameter configuration client object associated with the currently
            active controller
        :rtype: ControllerParamConfigClient obj (if None, returns False)
        
        :param controller_name: name of controller whose config client is required
        :type controller_name: str

        """
        if self._current_controller is None:
            rospy.logwarn("FrankaControllerManagerInterface: No active controller!")
            return False

        return self.get_controller_config_client(self._current_controller)


    def list_controller_names(self):
        """
        :return: List of names all controllers associated to a controller manager
            namespace. 
        :rtype: [str]
        
        :param only_motion_controller: if True, only motion controllers are returned
        :type only_motion_controller: bool

        """
        return [c.name for c in self.list_controllers()]



    """
        Properties that give the names of the controllers for each type.
    """
    @property
    def joint_velocity_controller(self):
        """
        :getter: Returns the name of joint velocity controller 
            (defined in franka_ros_controllers, and specified 
            in robot_config.yaml). Can be used for changing 
            motion controller using 
            :py:meth:`FrankaControllerManagerInterface.set_motion_controller`.
        :type: str
        """
        return self._controller_names_from_rosparam['joint_velocity_controller'] 
    @property
    def joint_position_controller(self):
        """
        :getter: Returns the name of joint position controller 
            (defined in franka_ros_controllers, and specified 
            in robot_config.yaml). Can be used for changing 
            motion controller using 
            :py:meth:`FrankaControllerManagerInterface.set_motion_controller`.
        :type: str
        """
        return self._controller_names_from_rosparam['joint_position_controller'] 
    @property
    def joint_torque_controller(self):
        """
        :getter: Returns the name of joint torque controller 
            (defined in franka_ros_controllers, and specified 
            in robot_config.yaml). Can be used for changing 
            motion controller using 
            :py:meth:`FrankaControllerManagerInterface.set_motion_controller`.
        :type: str
        """
        return self._controller_names_from_rosparam['joint_torque_controller']     
    @property
    def joint_impedance_controller(self):
        """
        :getter: Returns the name of joint impedance controller 
            (defined in franka_ros_controllers, and specified 
            in robot_config.yaml). Can be used for changing 
            motion controller using 
            :py:meth:`FrankaControllerManagerInterface.set_motion_controller`.
        :type: str
        """
        return self._controller_names_from_rosparam['joint_impedance_controller']  
    @property
    def effort_joint_position_controller(self):
        """
        :getter: Returns the name of effort-based joint position controller 
            (defined in franka_ros_controllers, and specified 
            in robot_config.yaml). Can be used for changing 
            motion controller using 
            :py:meth:`FrankaControllerManagerInterface.set_motion_controller`.
        :type: str
        """
        return self._controller_names_from_rosparam['effort_joint_position_controller']
    @property
    def joint_trajectory_controller(self):
        """
        :getter: Returns the name of joint trajectory controller 
            (defined in franka_ros_controllers, and specified 
            in robot_config.yaml). Can be used for changing 
            motion controller using 
            :py:meth:`FrankaControllerManagerInterface.set_motion_controller`.
            This controller exposes trajectory following service.
        :type: str
        """
        if self._in_sim:
            return self.joint_position_controller
        return self._controller_names_from_rosparam['joint_trajectory_controller']

    @property
    def current_controller(self):
        """
        :getter: Returns the name of currently active controller.
        :type: str
        """
        self._assert_one_active_controller()
        if not self._current_controller:
            rospy.logwarn("FrankaControllerManagerInterface: No active controller!")
        return self._current_controller

    @property
    def default_controller(self):
        """
        :getter: Returns the name of the default controller for Franka ROS Interface.
            (specified in robot_config.yaml). Should ideally default to the same as
            :py:meth:`joint_trajectory_controller`.
        :type: str
        """
        return self._controller_names_from_rosparam['default_controller']

    


if __name__ == '__main__':
    from franka_tools import FrankaFramesInterface
    cmi = FrankaControllerManagerInterface()
    f =  FrankaFramesInterface()





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

import dynamic_reconfigure.client


K_GAINS_KW = ['j1_k','j2_k','j3_k','j4_k','j5_k','j6_k','j7_k']
D_GAINS_KW = ['j1_d','j2_d','j3_d','j4_d','j5_d','j6_d','j7_d']


class ControllerParamConfigClient:
    """
        Interface class for updating dynamically configurable paramters of a controller.

        :param controller_name: The name of the controller.
        :type controller_name: str

    """
    def __init__(self, controller_name):
        """
        Initialisation: Client is not started yet.

        """
        self._controller_name = controller_name if controller_name[0] != '/' else controller_name[1:]
        self._is_running = False

    @property
    def is_running(self):
        """
        :return: True if client is running / server is unavailable; False otherwise
        :rtype: bool

        """
        return self._is_running
    

    def start(self,  timeout = 5):
        """
        Start the dynamic_reconfigure client
        
        :param timeout: time to wait before giving up on service request
        :type timeout: float

        """
        service_name = "/{}/arm/controller_parameters_config".format(self._controller_name)
        try:
            self._client = dynamic_reconfigure.client.Client(service_name, timeout=timeout, config_callback=self._log_update)
        except rospy.ROSException:
            rospy.logdebug("ControllerParamConfigClient: Could not find configuration server at {}".format(service_name))
        self._is_running = True

    def _log_update(self, config):  
        """
            Optional callback to log parameter changes after each update request.

        """
        k_gains = [config[n] for n in K_GAINS_KW]
        d_gains = [config[n] for n in D_GAINS_KW]

        rospy.logdebug("ControllerParamConfigClient: {controller_name} config set to \n\tSmoothing Param: {smoothing_param} \n\tJoint Stiffness: {K_gains} \n\tJoint Damping: {D_gains}".format(controller_name=self._controller_name, smoothing_param = config['position_joint_delta_filter'], K_gains = k_gains, D_gains = d_gains))


    def update_config(self, **kwargs):
        """
        Update the config in the server using the provided keyword arguments.
        
        :param kwargs: These are keyword arguments matching the parameter names
            in config file: franka_ros_controllers/cfg/joint_controller_params.cfg

        """
        self._client.update_configuration(kwargs)


    def set_controller_gains(self, k_gains, d_gains = None):
        """
        Update the stiffness and damping parameters of the joints for the current controller.
        
        :param k_gains: joint stiffness parameters (should be within limits specified in 
                        franka documentation; same is also set 
                        in franka_ros_controllers/cfg/joint_controller_params.cfg)
        :type k_gains: [float]
        :param d_gains: joint damping parameters (should be within limits specified in 
                        franka documentation; same is also set 
                        in franka_ros_controllers/cfg/joint_controller_params.cfg)
        :type d_gains: [float]

        """
        assert len(k_gains) == 7, "ControllerParamConfigClient: k_gains argument should be of length 7!"

        config = {}
        for i, k_val in enumerate(k_gains):
            config[K_GAINS_KW[i]] = k_val

        if d_gains:
            assert len(k_gains) == 7, "ControllerParamConfigClient: d_gains argument should be of length 7!"

            for j, d_val in enumerate(d_gains):
                config[D_GAINS_KW[j]] = d_val

        rospy.logdebug("Config: {}".format(config))

        self.update_config(**config)


    def set_joint_motion_smoothing_parameter(self, value):
        """
        Update the joint motion smoothing parameter (only valid for 
            position_joint_position_controller).
        
        :param value: smoothing factor (should be within limit set 
                      in franka_ros_controllers/cfg/joint_controller_params.cfg)
        :type value: [float]

        """
        self.update_config(position_joint_delta_filter = value)

    def get_joint_motion_smoothing_parameter(self, timeout = 5):
        """
        :return: the currently set value for the joint position smoothing parameter from 
            the server.
        :rtype: float
        
        :param timeout: time to wait before giving up on service request
        :type timeout: float

        """
        return self.get_config(timeout = timeout)['position_joint_delta_filter']


    def get_config(self, timeout = 5):
        """
        :return: the currently set values for all paramters from the server
        :rtype: dict {str : float}
        
        :param timeout: time to wait before giving up on service request
        :type timeout: float

        """
        return self._client.get_configuration(timeout = timeout)

    def get_controller_gains(self, timeout = 5):
        """
        :return: the currently set values for controller gains from the server
        :rtype: ( [float], [float] )
        
        :param timeout: time to wait before giving up on service request
        :type timeout: float

        """
        config = self.get_config(timeout = timeout)

        k_gains = []

        for k_val_ in K_GAINS_KW:
            if k_val_ in config:
                k_gains.append(config[k_val_])
            else:
                rospy.logwarn("ControllerParamConfigClient: Could not find K gain {} in server".format(k_val_))
                return False, False

        d_gains = []

        for d_val_ in D_GAINS_KW:
            if d_val_ in config:
                d_gains.append(config[d_val_])
            else:
                rospy.logwarn("ControllerParamConfigClient: Could not find D gain {} in server".format(d_val_))
                return k_gains, False

        return k_gains, d_gains
                


    def get_parameter_descriptions(self, timeout = 5):
        """
        :return: the description of each parameter as defined in the cfg
            file from the server.
        :rtype: dict {str : str}
        
        :param timeout: time to wait before giving up on service request
        :type timeout: float

        """
        return self._client.get_parameter_descriptions(timeout = timeout)


if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    # client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)


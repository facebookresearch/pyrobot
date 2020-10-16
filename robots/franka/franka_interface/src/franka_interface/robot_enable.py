# /***************************************************************************

# 
# @package: franka_interface
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

"""
 @info: 
       Wrapper class for enabling and resetting robot state.

"""


import rospy
from threading import Lock

from franka_control.msg import ErrorRecoveryActionGoal
from franka_core_msgs.msg import RobotState

import franka_dataflow
from robot_params import RobotParams

class RobotEnable(object):
    """
    Class RobotEnable - simple control/status wrapper around robot state

    enable()  - enable all joints
    disable() - disable all joints
    reset()   - reset all joints, reset all jrcp faults, disable the robot
    stop()    - stop the robot, similar to hitting the e-stop button

    :param robot_params: A RobotParams instance (optional)
    :type robot_params: RobotParams
    """

    param_lock = Lock()

    def __init__(self, robot_params = None):
        """
        
        """

        if robot_params:
            self._params = robot_params
        else:
            self._params = RobotParams()

        self._ns = self._params.get_base_namespace()

        self._enabled = None

        state_topic = '{}/custom_franka_state_controller/robot_state'.format(self._ns)
        self._state_sub = rospy.Subscriber(state_topic,
                                           RobotState,
                                           self._state_callback
                                           )

        franka_dataflow.wait_for(
            lambda: not self._enabled is None,
            timeout=5.0,
            timeout_msg=("Failed to get robot state on %s" %
            (state_topic,)),
        )

    def _state_callback(self, msg):
        self._enabled = (msg.robot_mode != 4) 

    def is_enabled(self):
        """
        Return status of robot

        :return: True if enabled, False otherwise
        :rtype: bool
        """
        return self._enabled
    

    def _toggle_enabled(self, status):

        pub = rospy.Publisher('{}/franka_control/error_recovery/goal'.format(self._ns), ErrorRecoveryActionGoal,
                              queue_size=10)

        if self._enabled == status:
            rospy.loginfo("Robot is already %s"%self.state())

        franka_dataflow.wait_for(
            test=lambda: self._enabled == status,
            timeout=5.0,
            timeout_msg=("Failed to %sable robot" %
                         ('en' if status else 'dis',)),
            body=lambda: pub.publish(ErrorRecoveryActionGoal()),
        )
        rospy.loginfo("Robot %s", ('Enabled' if status else 'Disabled'))

    def state(self):
        """
        Returns the last known robot state.

        :rtype: str
        :return: "Enabled"/"Disabled"
        """
        return "%sabled"%('en' if self._enabled else 'dis',)

    def enable(self):
        """
        Enable all joints
        """
        if not self._enabled:
            rospy.loginfo("Robot Stopped: Attempting Reset...")
            self._toggle_enabled(True)

    def disable(self):
        """
        Disable all joints
        """
        self._toggle_enabled(False)
        
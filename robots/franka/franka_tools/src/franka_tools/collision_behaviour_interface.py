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
from franka_control.srv import SetForceTorqueCollisionBehavior, SetFullCollisionBehavior

DEFAULT_VALUES =  {
            'torque_acc_lower'  : [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            'torque_acc_upper'  : [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            'torque_lower'      : [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            'torque_upper'      : [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            'force_acc_lower'   : [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            'force_acc_upper'   : [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            'force_lower'   : [5.0, 5.0, 5.0, 25.0, 25.0, 25.0],
            'force_upper'   : [20.0, 20.0, 20.0, 25.0, 25.0, 25.0] # NOTE: COLLISION VALUES HAVE TO BE HIGHER THAN CONTACT THRESHOLDS (I.E. 'UPPER' THRESHOLDS HAVE TO BE HIGHER THAN 'LOWER' THRESHOLDS) FOR DETECTING COLLISIONS CORRECTLY. THE ROBOT DETECTS THE HIGHER VALUE OF THE TWO AS COLLISION.
} 

class CollisionBehaviourInterface(object):

    """
        Helper class to set collision and contact thresholds at cartesian and joint levels.
        (This class has no 'getter' functions to access the currently set collision behaviour valures.)
    """

    def __init__(self):

        rospy.loginfo("Waiting for collision behaviour services...")
        rospy.wait_for_service("/franka_ros_interface/franka_control/set_force_torque_collision_behavior", timeout = 1.0)
        self._ft_collision_behaviour_handle = rospy.ServiceProxy("/franka_ros_interface/franka_control/set_force_torque_collision_behavior", SetForceTorqueCollisionBehavior)
        self._full_collision_behaviour_handle = rospy.ServiceProxy("/franka_ros_interface/franka_control/set_full_collision_behavior", SetForceTorqueCollisionBehavior)

        rospy.loginfo("Collision behaviour services found.")

    def set_ft_contact_collision_behaviour(self, torque_lower = None, torque_upper = None, force_lower = None, force_upper = None):
        """
        :return: True if service call successful, False otherwise
        :rtype: bool
        :param torque_lower: Joint torque threshold for contact detection
        :type torque_lower: [float] size 7
        :param torque_upper: Joint torque threshold for collision (robot motion stops if violated)
        :type torque_upper: [float] size 7
        :param force_lower: Cartesian force threshold for contact detection [x,y,z,R,P,Y]
        :type force_lower: [float] size 6
        :param force_upper: Cartesian force threshold for collision detection [x,y,z,R,P,Y] (robot motion stops if violated)
        :type force_upper: [float] size 6
        """

        if torque_lower is None:
            torque_lower = DEFAULT_VALUES['torque_lower']
        if torque_upper is None:
            torque_upper = DEFAULT_VALUES['torque_upper']
        if force_lower is None:
            force_lower = DEFAULT_VALUES['force_lower']
        if force_upper is None:
            force_upper = DEFAULT_VALUES['force_upper']

        try:
            response = self._ft_collision_behaviour_handle(lower_torque_thresholds_nominal = torque_lower,
                                                           upper_torque_thresholds_nominal = torque_upper,
                                                           lower_force_thresholds_nominal  = force_lower,
                                                           upper_force_thresholds_nominal  = force_upper)

            rospy.loginfo("CollisionBehaviourInterface: Collision behaviour change request: %s. \n\tDetails: %s"%("Success" if response.success else "Failed!", response.error))

        except rospy.ServiceException, e:
            rospy.logwarn("CollisionBehaviourInterface: Collision behaviour change service call failed: %s"%e)
            return False

    def set_force_threshold_for_contact(self, cartesian_force_values):
        """
        :return: True if service call successful, False otherwise
        :rtype: bool
        :param cartesian_force_values: Cartesian force threshold for contact detection [x,y,z,R,P,Y]
        :type cartesian_force_values: [float] size 6
        """
        return self.set_ft_contact_collision_behaviour(force_lower = cartesian_force_values)

    def set_force_threshold_for_collision(self, cartesian_force_values):
        """
        :return: True if service call successful, False otherwise
        :rtype: bool
        :param cartesian_force_values: Cartesian force threshold for collision detection [x,y,z,R,P,Y] (robot motion stops if violated)
        :type cartesian_force_values: [float] size 6
        """
        return self.set_ft_contact_collision_behaviour(force_upper = cartesian_force_values)

    def set_collision_threshold(self, joint_torques = None, cartesian_forces = None):
        """
        :return: True if service call successful, False otherwise
        :rtype: bool
        :param joint_torques: Joint torque threshold for collision (robot motion stops if violated)
        :type joint_torques: [float] size 7
        :param cartesian_forces: Cartesian force threshold for collision detection [x,y,z,R,P,Y] (robot motion stops if violated)
        :type cartesian_forces: [float] size 6
        """
        return self.set_ft_contact_collision_behaviour(torque_upper = joint_torques, force_upper = cartesian_forces)

    def set_contact_threshold(self, joint_torques = None, cartesian_forces = None):
        """
        :return: True if service call successful, False otherwise
        :rtype: bool
        :param joint_torques: Joint torque threshold for identifying as contact
        :type joint_torques: [float] size 7
        :param cartesian_forces: Cartesian force threshold for identifying as contact
        :type cartesian_forces: [float] size 6
        """
        return self.set_ft_contact_collision_behaviour(torque_lower = joint_torques, force_lower = cartesian_forces)

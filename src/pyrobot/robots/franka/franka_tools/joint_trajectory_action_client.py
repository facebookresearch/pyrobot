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
import sys
import actionlib
from copy import copy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


class JointTrajectoryActionClient(object):
    """
    To use this class, the currently active controller for the franka robot should be
    the "joint_position_trajectory_controller". This can be set using instance of
    :py:class:`franka_tools.FrankaControllerManagerInterface`.
    """
    def __init__(self, joint_names, controller_name = "position_joint_trajectory_controller"):
        self._joint_names = joint_names

        self._client = actionlib.SimpleActionClient("/%s/follow_joint_trajectory"%(controller_name),
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(3.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time, velocities = None):
        """
        Add a waypoint to the trajectory.

        :param positions: target joint positions
        :type positions: [float]*7
        :param time: target time in seconds from the start of trajectory 
            to reach the specified goal 
        :type time: float
        :param velocities: goal velocities for joints (give atleast 0.0001)
        :type velocities: [float]*7

        .. note:: Velocities should be greater than zero (done by default) for
            smooth motion.

        """
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        if velocities is None:
            point.velocities = [0.0001 for n in positions]
        else:
            point.velocities = copy(velocities)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        """
        Execute previously defined trajectory.
        """
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        """
        Stop currently executing trajectory.
        """
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        """
        Wait for trajectory execution result.

        :param timeout: timeout before cancelling wait
        :type timeout: float
        """
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        """
        Get result from trajectory action server
        """
        return self._client.get_result()

    def clear(self):
        """
        Clear all waypoints from the current trajectory definition.
        """
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names


if __name__ == '__main__':
    
    pass
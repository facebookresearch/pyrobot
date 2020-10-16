# /***************************************************************************

# 
# @package: franka_moveit
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
import moveit_commander

class ExtendedPlanningSceneInterface(moveit_commander.PlanningSceneInterface):
    """
    .. note:: For other available methods for planning scene interface, refer `PlanningSceneInterface <http://docs.ros.org/indigo/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html>`_.
    """
    def __init__(self):

        moveit_commander.PlanningSceneInterface.__init__(self)


    def add_box(self, name, pose, size, timeout = 5):
        """
        Add object to scene and check if it is created.

        :param name: name of object
        :param pose: desired pose for the box (Use :py:func:`franka_moveit.utils.create_pose_stamped_msg`)
        :param size: size of the box
        :param timeout: time in sec to wait while checking if box is created 

        :type name: str
        :type pose: geometry_msgs.msg.PoseStamped
        :type size: [float] (len 3)
        :type timeout: float
        """

        moveit_commander.PlanningSceneInterface.add_box(self, name = name, pose = pose, size=size)
        return self._wait_for_state_update(object_name = name, object_is_known=True, timeout=timeout)



    def _wait_for_state_update(self, object_name, object_is_known=False, object_is_attached=False, timeout=5):

        start = rospy.get_time()
        while (rospy.get_time() - start < timeout) and not rospy.is_shutdown():

            attached_objects = self.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = object_name in self.get_known_object_names()

            if (object_is_attached == is_attached) and (object_is_known == is_known):
                return True

            rospy.sleep(0.1)
        return False

    def remove_box(self, box_name, timeout=5):
        """
        Remove box from scene.

        :param box_name: name of object
        :param timeout: time in sec to wait while checking if box is created 

        :type box_name: str
        :type timeout: float

        """
        self.remove_world_object(box_name)

        return self._wait_for_state_update(object_name = box_name, object_is_attached=False, object_is_known=False, timeout=timeout)




if __name__ == '__main__':
    scene = ExtendedPlanningSceneInterface()

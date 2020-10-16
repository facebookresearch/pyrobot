#! /usr/bin/env python

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

import sys
import rospy
import moveit_commander

from franka_moveit import ExtendedPlanningSceneInterface
from franka_moveit.utils import create_pose_stamped_msg

"""
A script for creating a simple environment as a PlanningScene
"""

IRLab_workspace = [
           {
           'name': 'back_wall',
           'pose': create_pose_stamped_msg(position = [-0.57,0.0,0.5], orientation = [1,0,0,0], frame = 'panda_link0'),
           'size': [0.1,1.8,1]
           },
           {
           'name': 'side_wall',
           'pose': create_pose_stamped_msg(position = [-0.3,-0.85,0.5], orientation = [1,0,0,0], frame = 'panda_link0'),
           'size': [0.6,0.1,1]
           },
           {
           'name': 'table',
           'pose': create_pose_stamped_msg(position = [0.45,-0.0,0], orientation = [1,0,0,0], frame = 'panda_link0'),
           'size': [2,1.8,0.02]
           },
           {
           'name': 'controller_box',
           'pose': create_pose_stamped_msg(position = [-0.37,0.55,0.08], orientation = [1,0,0,0], frame = 'panda_link0'),
           'size': [0.4,0.6,0.16]
           },
           {
           'name': 'equipment_box',
           'pose': create_pose_stamped_msg(position = [-0.35,-0.68,0.17], orientation = [1,0,0,0], frame = 'panda_link0'),
           'size': [0.46,0.4,0.34]
           }
            ]



def main():
  try:
    rospy.loginfo("Creating Demo Planning Scene")
    scene = ExtendedPlanningSceneInterface()

    rospy.sleep(1) # ----- Not having this delay sometimes caused failing to create some boxes

    for config in IRLab_workspace:

        rospy.loginfo("-- Creating object: {}..".format(config['name']))
        success = scene.add_box(**config)
        rospy.loginfo("------ {}".format("success" if success else "FAILED!"))

    rospy.loginfo("Created Demo Planning Scene.")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    rospy.init_node('simple_scene_creator',
                    anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    main()


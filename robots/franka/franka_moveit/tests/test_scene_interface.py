#! /usr/bin/env python

import sys
import rospy
import moveit_commander

from franka_moveit import ExtendedPlanningSceneInterface
from franka_moveit.utils import create_pose_stamped_msg

"""
    Test if sceneinterface works. Use rviz and add PlanningScene visual while running the interface.
"""

objects = [
           {
           'name': 'box1',
           'pose': create_pose_stamped_msg(position = [0,0,0], orientation = [1,0,0,0], frame = "world"), 
           'size': [0.1,0.1,0.1]
           } ,
           {
           'name': 'box2',
           'pose': create_pose_stamped_msg(position = [0.5,0,0], orientation = [1,0,0,0], frame = "world"),
           'size': [0.2,0.2,0.2]
           },
           {
           'name': 'box3',
           'pose': create_pose_stamped_msg(position = [0.4,-0.2,0.5], orientation = [1,0,0,0], frame = "world"),
           'size': [0.1,0.1,0.1]
           }
            ]



def main():

  print "============ Starting test ..."
  rospy.sleep(5.)
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the scene_interface (press ctrl-d to exit) ..."
    raw_input()
    scene = ExtendedPlanningSceneInterface()

    print "============ Press `Enter` to add objects to the planning scene ..."
    raw_input()
    for config in objects:
        scene.add_box(**config)

    print "============ Press `Enter` to remove the boxes from the planning scene ..."
    raw_input()
    for config in objects:
        scene.remove_box(config['name'])

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    rospy.init_node('simple_scene_tester',
                    anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    main()


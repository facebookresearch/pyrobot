#! /usr/bin/env python

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
   NOTE: Running this script without any arguments will stop the current command being 
         sent to the gripper. Use this to stop any dangerous action being run
   Utility script for simple gripper motion.
     Usage:
         simple_gripper.py <arg>

     @Args:
         <No arg>          : Stop current gripper action
         -o (--open)       : optional argument to make the gripper release and go to max width 
         -c (--close)      : optional argument to close the gripper 

"""


import rospy
import argparse
import actionlib

from franka_interface import GripperInterface
from franka_gripper.msg import ( StopAction, StopGoal )

def _active_cb():
    rospy.loginfo("GripperInterface: '{}' request active.".format(_caller))

def _feedback_cb(msg):
    rospy.loginfo("GripperInterface: '{}' request feedback: \n\t{}".format(_caller,msg))

def _done_cb(status, result):
    rospy.loginfo("GripperInterface: '{}' complete. Result: \n\t{}".format(_caller, result))

if __name__ == '__main__':
    
    rospy.init_node("gripper_stop_node")

    _caller = "stop_gripper"
    stop_action_client = actionlib.SimpleActionClient("/franka_ros_interface/franka_gripper/stop", StopAction)
    stop_action_client.wait_for_server()

    stop_action_client.send_goal(StopGoal(), done_cb =_done_cb, active_cb = _active_cb, feedback_cb = _feedback_cb)

    result = stop_action_client.wait_for_result(rospy.Duration(15.))

    parser = argparse.ArgumentParser("Stop current gripper action; Open or close gripper.")

    gripper_group = parser.add_mutually_exclusive_group(required=False)
    gripper_group.add_argument("-o", "--open", dest="open",
        action='store_true', default=False, help="open gripper")
    gripper_group.add_argument("-c", "--close", dest="close",
        action='store_true', default=False, help="close gripper")
    args = parser.parse_args(rospy.myargv()[1:])

    if args.open or args.close:

        gi = GripperInterface()

        if args.open:
            gi.open()
        else:
            gi.close()


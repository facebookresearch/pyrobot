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
# Copyright (c) 2013-2018, Rethink Robotics Inc.
 
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

from franka_interface import ArmInterface, GripperInterface
from franka_dataflow.getch import getch


def map_keyboard():
    """
        Map keyboard keys to robot joint motion. Keybindings can be 
        found when running the script.
    """

    limb = ArmInterface()

    gripper = GripperInterface(ns = limb.get_robot_params().get_base_namespace())

    has_gripper = gripper.exists

    joints = limb.joint_names()

    def set_j(limb, joint_name, delta):
        joint_command = limb.joint_angles()
        joint_command[joint_name] += delta
        limb.set_joint_positions(joint_command)

    def set_g(action):
        if has_gripper:
            if action == "close":
                gripper.close()
            elif action == "open":
                gripper.open()
            elif action == "calibrate":
                gripper.calibrate()

    bindings = {
        '1': (set_j, [limb, joints[0], 0.1], joints[0]+" increase"),
        'q': (set_j, [limb, joints[0], -0.1], joints[0]+" decrease"),
        '2': (set_j, [limb, joints[1], 0.1], joints[1]+" increase"),
        'w': (set_j, [limb, joints[1], -0.1], joints[1]+" decrease"),
        '3': (set_j, [limb, joints[2], 0.01], joints[2]+" increase"),
        'e': (set_j, [limb, joints[2], -0.01], joints[2]+" decrease"),
        '4': (set_j, [limb, joints[3], 0.01], joints[3]+" increase"),
        'r': (set_j, [limb, joints[3], -0.01], joints[3]+" decrease"),
        '5': (set_j, [limb, joints[4], 0.01], joints[4]+" increase"),
        't': (set_j, [limb, joints[4], -0.01], joints[4]+" decrease"),
        '6': (set_j, [limb, joints[5], 0.01], joints[5]+" increase"),
        'y': (set_j, [limb, joints[5], -0.01], joints[5]+" decrease"),
        '7': (set_j, [limb, joints[6], 0.01], joints[6]+" increase"),
        'u': (set_j, [limb, joints[6], -0.01], joints[6]+" decrease")
     }
    if has_gripper:
        bindings.update({
        '8': (set_g, "close", "close gripper"),
        'i': (set_g, "open", "open gripper"),
        '9': (set_g, "calibrate", "calibrate gripper")
        })
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                if c == '8' or c == 'i' or c == '9':
                    cmd[0](cmd[1])
                    print("command: %s" % (cmd[2],))
                else:
                    #expand binding to something like "set_j(right, 'j0', 0.1)"
                    cmd[0](*cmd[1])
                    print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def main():
    """Franka ROS Interface Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on the robot arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """

    print("Initializing node... ")
    rospy.init_node("fri_example_joint_position_keyboard")
    print("Getting robot state... ")

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()

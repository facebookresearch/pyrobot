# *******************************************************************************
# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
# *******************************************************************************
import sys

import moveit_commander
import numpy as np
import rospy

import argparse, copy, sys, select, signal, termios, tty, time
import rospy, moveit_commander, tf, tf_conversions
import numpy as np

from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64

from std_srvs.srv import Empty

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# from IPython import embed
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK, GetPositionFK

# from trac_ik_python.trac_ik import IK
from dynamixel_workbench_msgs.srv import JointCommand
from analytic_ik import AnalyticInverseKinematics


class GazeboInterface:
    """An example class on how to inteface with Gazebo environment"""

    def __init__(self):
        rospy.init_node("gazebo_keyboard_teleoperation", anonymous=True)

        # subscriber to get joint angles froim gazebo
        rospy.Subscriber("/joint_state", JointState, self._callback_joint_states)

        # listener for transoforms
        self.listener = tf.TransformListener()

        # publishers for joint angles to gazebo
        self.head_pan_pub = rospy.Publisher(
            "/head_pan_cntrl/command", Float64, queue_size=1
        )
        self.head_tilt_pub = rospy.Publisher(
            "/head_tilt_cntrl/command", Float64, queue_size=1
        )
        self.joint_1_pub = rospy.Publisher(
            "/joint_1_cntrl/command", Float64, queue_size=1
        )
        self.joint_2_pub = rospy.Publisher(
            "/joint_2_cntrl/command", Float64, queue_size=1
        )
        self.joint_3_pub = rospy.Publisher(
            "/joint_3_cntrl/command", Float64, queue_size=1
        )
        self.joint_4_pub = rospy.Publisher(
            "/joint_4_cntrl/command", Float64, queue_size=1
        )
        self.joint_5_pub = rospy.Publisher(
            "/joint_5_cntrl/command", Float64, queue_size=1
        )
        self.joint_6_pub = rospy.Publisher(
            "/joint_6_cntrl/command", Float64, queue_size=1
        )
        self.joint_7_pub = rospy.Publisher(
            "/joint_7_cntrl/command", Float64, queue_size=1
        )

        # service to reset the gazebo world
        rospy.wait_for_service("/gazebo/reset_world")
        self.reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

        # service to reset the gazebo robot
        rospy.wait_for_service("/gazebo/reset_simulation")
        self.reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)

        # services to pause and unpause the simulations
        rospy.wait_for_service("/gazebo/pause_physics")
        self.pause_sim = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.unpause_sim = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

    def _callback_joint_states(self, data):
        self._joint_states = data

    def set_robot(self, joint_target):  # similar order as seen in topic /joint_states

        self.head_pan_pub.publish(joint_target[0])
        self.head_tilt_pub.publish(joint_target[1])
        self.joint_1_pub.publish(joint_target[2])
        self.joint_2_pub.publish(joint_target[3])
        self.joint_3_pub.publish(joint_target[4])
        self.joint_4_pub.publish(joint_target[5])
        self.joint_5_pub.publish(joint_target[6])
        self.joint_6_pub.publish(joint_target[7])
        self.joint_7_pub.publish(joint_target[8])

    def set_arm(self, joint_target):
        self.joint_1_pub.publish(joint_target[0])
        self.joint_2_pub.publish(joint_target[1])
        self.joint_3_pub.publish(joint_target[2])
        self.joint_4_pub.publish(joint_target[3])
        self.joint_5_pub.publish(joint_target[4])

    def reset_robot(self):
        # rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_simulation()

    def pause_simulation(self):
        # rospy.wait_for_service('/gazebo/pause_physics')
        self.pause_sim()

    def resume_simulation(self):
        # rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause_sim()


if __name__ == "__main__":

    server = GazeboInterface()

    rospy.sleep(2)
    # set all the angles to zero
    print("setting the provided joint angles...")
    server.set_robot(np.zeros(9))

    rospy.sleep(2)

    # reset simulation
    print("Resetting the simulation...")
    server.reset_robot()

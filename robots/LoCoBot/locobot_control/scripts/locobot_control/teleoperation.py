#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Client and server for keyboard teleoperation
"""
import copy
import os
import sys
import termios
import time
import tty

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Twist
from locobot_control.srv import JointCommand
from pyrobot import Robot
from std_msgs.msg import Empty, String, Bool
from tf.transformations import euler_from_quaternion

KEY_CTRL_C = "\x03"
KEY_BASE_FORWARD = "\x1b[A"
KEY_BASE_BACKWARD = "\x1b[B"
KEY_BASE_TURNRIGHT = "\x1b[C"
KEY_BASE_TURNLEFT = "\x1b[D"

HELP_MSG = """
Teleoperate your LoCoBot!
---------------------------
Pre-requisite:
roslaunch locobot_control teleoperation.launch

Moving around:

1) End-effector
           w (+x)
  a (+y)   s (-x)  d (-y)
        z (+z)   x (-z)

2) Joint 4 (wrist link)
        h (+)    j(-)

3) Joint 5 (gripper link)
        k (+)    l(-)

4) Gripper
    n (Open)    m (Close)

5) Pan-Tilt Camera
    u (+pan)  i (-pan)  o (+tilt)  p (-tilt)

6) Move Base (Arrow Keys)
    up (move forward) down (move backward)
    left (turn left)  right (turn right) 
    
CTRL-C to quit
"""


class RobotTeleoperationServer:
    def __init__(self, cfg_file="teleop.yaml"):

        # Load config file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        root_path = os.path.dirname(os.path.dirname(dir_path))
        cfg_path = os.path.join(root_path, "config", cfg_file)
        with open(cfg_path, "r") as f:
            self.cfg = yaml.load(f)
        arm_config = {"control_mode": "position", "use_moveit": False}

        self.use_arm = rospy.get_param("use_arm")
        self.use_camera = rospy.get_param("use_camera")
        self.use_base = rospy.get_param("use_base")
        self.use_gripper = self.use_arm

        self.bot = Robot(
            "locobot",
            use_arm=self.use_arm,
            use_base=self.use_base,
            use_camera=self.use_camera,
            use_gripper=self.use_gripper,
            arm_config=arm_config,
        )

        # Subscribers
        rospy.Subscriber(self.cfg["ROSTOPIC_TELEOP"], String, self._callback_cmd_vel)
        rospy.Subscriber(
            self.cfg["ROSTOPIC_HARDWARE_STATUS"], Bool, self._callback_hardware_status
        )

        # Publishers
        self.pub_stop = rospy.Publisher(
            self.cfg["ROSTOPIC_STOP_EXECUTION"], Empty, queue_size=1
        )
        self.pub_stop_j4 = rospy.Publisher(
            self.cfg["ROSTOPIC_STOP_EXECUTION_J4"], Empty, queue_size=1
        )
        self.pub_stop_j5 = rospy.Publisher(
            self.cfg["ROSTOPIC_STOP_EXECUTION_J5"], Empty, queue_size=1
        )
        self.base_cmd_pub = rospy.Publisher(
            self.cfg["ROSTOPIC_BASE_COMMAND"], Twist, queue_size=1
        )
        self.safety_status_pub = rospy.Publisher(
            self.cfg["ROSTOPIC_SAFETY_STATUS"], Bool, queue_size=1
        )

        # Services
        self.joint_cmd_srv = rospy.ServiceProxy(
            self.cfg["ROSTOPIC_JOINT_COMMAND"], JointCommand
        )

        # Initialize arm, gripper and pan-tilt camera
        if self.use_arm:
            # set arm to a good pose
            # if the arm starts from the rest pose,
            # some teleops might break the arm
            self.bot.arm.set_joint_positions(
                np.array([0, -0.14, 0.76, 1.07, 0]), plan=False
            )
        # NOTE - Do not remove the following steps!
        if self.use_gripper:
            self.bot.gripper.reset()
        if self.use_camera:
            self.bot.camera.set_pan(self.cfg["START_PAN"])
            self.bot.camera.set_tilt(self.cfg["START_TILT"])

        self.cmd = None
        self.status = True
        self.cmd_prev = None
        self.is_robot_moving = False
        self.base_linear_speed = 0.05
        self.base_angular_speed = 0.5
        self.start_time = time.time()

        self.target_alpha = 0.0
        self.update_alpha()
        self.trans, _, _ = self.bot.arm.pose_ee

        rospy.loginfo("Teleop server ready to accept requests...")

    def _callback_cmd_vel(self, data):
        self.cmd = data.data

    def _callback_hardware_status(self, data):
        self.status = data.data

    def update_alpha(self):
        _, _, quat = self.bot.arm.get_ee_pose("/base_link")
        (_, pitch, _) = euler_from_quaternion(quat)
        self.target_alpha = -pitch

    @property
    def pose_fingertip(self):
        """ pose of finger tip """
        return self.bot.arm.get_transform("/base_link", "/finger_l")

    def set_joint_motor(self, joint_id, joint_angle):
        self.is_robot_moving = True
        self.start_time = time.time()
        self.joint_cmd_srv("rad", joint_id, joint_angle)

    def set_joint(self, joint_target):
        self.is_robot_moving = True
        self.start_time = time.time()
        self.bot.arm.set_joint_positions(joint_target, plan=False, wait=False)

    def move_joint_individual(self, joint_id, increment=True):
        sign = 1 if increment else -1
        self.set_joint_motor(
            joint_id,
            self.bot.arm.get_joint_angle("joint_" + str(joint_id))
            + sign * self.cfg["DELTA_ANGLE_" + str(joint_id)],
        )
        if joint_id == 4:
            (trans, _, _) = self.bot.arm.pose_ee
            self.trans = trans.copy()

    def move_pan(self, increment=True):
        sign = 1 if increment else -1
        assert (
            self.use_camera
        ), "Please set `use_camera:=true` when you launch the robot driver"
        self.bot.camera.set_pan(
            self.bot.camera.get_pan() + sign * self.cfg["DELTA_ANGLE_CAMERA"],
            wait=False,
        )

    def move_tilt(self, increment=True):
        sign = 1 if increment else -1
        assert (
            self.use_camera
        ), "Please set `use_camera:=true` when you launch the robot driver"
        self.bot.camera.set_tilt(
            self.bot.camera.get_tilt() + sign * self.cfg["DELTA_ANGLE_CAMERA"],
            wait=False,
        )

    def check_safety(self, key):
        """
        Simple heuristics for avoiding collisions
        """

        # check the shoulder angle
        if self.bot.arm.get_joint_angle("joint_1") < -0.6:
            rospy.logerr(
                "Possible risk of collision with camera mount when moving back"
            )
            self.safety_status_pub.publish(False)
            return False

        # check if end effector is too close to the floor
        if (
            self.pose_fingertip[0][-1] < self.cfg["COLLISION_Z_THRESHOLD"]
            and not key == self.cfg["KEY_POS_Z"]
        ):
            rospy.logerr("Possible risk of collision with floor")
            self.safety_status_pub.publish(False)
            return False

        self.safety_status_pub.publish(True)
        return True

    def move_arm(self, delta, key):
        target_position = self.trans.copy().flatten()
        target_position += delta
        result = self.bot.arm.set_ee_pose_pitch_roll(
            target_position, -self.target_alpha, plan=False, numerical=False
        )
        if result:
            self.trans = target_position.reshape(3, 1).copy()

        self.check_safety(key)

    def move_base(self, lin_speed, ang_speed):
        if self.use_base:
            if np.fabs(lin_speed) > 0.01 or np.fabs(ang_speed) > 0.01:
                self.is_robot_moving = True
                self.bot.base.set_vel(
                    fwd_speed=lin_speed, turn_speed=ang_speed, exe_time=0.5
                )
            else:
                self.bot.base.stop()

    def reset(self):
        self.set_joint(np.zeros(5))

    def run(self):
        rospy.sleep(1)
        while True:
            key = copy.deepcopy(self.cmd)
            if key is None:
                if (
                    self.is_robot_moving
                    and time.time() - self.start_time > self.cfg["WAIT_TIME"]
                ):
                    if self.cmd_prev in [
                        self.cfg["KEY_POS_J5"],
                        self.cfg["KEY_NEG_J5"],
                    ]:
                        self.pub_stop_j5.publish()
                    elif self.cmd_prev in [
                        self.cfg["KEY_POS_J4"],
                        self.cfg["KEY_NEG_J4"],
                    ]:
                        self.pub_stop_j4.publish()
                    else:
                        pass
                    self.move_base(0, 0)
                    self.is_robot_moving = False
                continue
            if key != self.cmd_prev:

                if self.cmd_prev in [
                    self.cfg["KEY_POS_J5"],
                    self.cfg["KEY_NEG_J5"],
                    self.cfg["KEY_POS_J4"],
                    self.cfg["KEY_NEG_J4"],
                ]:
                    self.update_alpha()

            delta = np.zeros(3)

            if not self.status:
                rospy.logerr(
                    "Arm has been shutdown, " "there is some error, check logs"
                )
                self.exit()

            if key == self.cfg["KEY_POS_X"]:
                delta[0] += self.cfg["DELTA_POS_X"]
                self.move_arm(delta, key)
            elif key == self.cfg["KEY_NEG_X"]:
                delta[0] += self.cfg["DELTA_NEG_X"]
                self.move_arm(delta, key)
            elif key == self.cfg["KEY_POS_Y"]:
                delta[1] += self.cfg["DELTA_POS_Y"]
                self.move_arm(delta, key)
            elif key == self.cfg["KEY_NEG_Y"]:
                delta[1] += self.cfg["DELTA_NEG_Y"]
                self.move_arm(delta, key)
            elif key == self.cfg["KEY_POS_Z"]:
                delta[2] += self.cfg["DELTA_POS_Z"]
                self.move_arm(delta, key)
            elif key == self.cfg["KEY_NEG_Z"]:
                delta[2] += self.cfg["DELTA_NEG_Z"]
                self.move_arm(delta, key)
            elif key == self.cfg["KEY_POS_J5"]:
                self.move_joint_individual(5)
            elif key == self.cfg["KEY_NEG_J5"]:
                self.move_joint_individual(5, False)
            elif key == self.cfg["KEY_POS_J4"]:
                self.move_joint_individual(4)
            elif key == self.cfg["KEY_NEG_J4"]:
                self.move_joint_individual(4, False)
            elif key == KEY_CTRL_C:
                self.exit()
            elif key == self.cfg["KEY_POS_PAN"]:
                self.move_pan()
            elif key == self.cfg["KEY_NEG_PAN"]:
                self.move_pan(False)
            elif key == self.cfg["KEY_POS_TILT"]:
                self.move_tilt()
            elif key == self.cfg["KEY_NEG_TILT"]:
                self.move_tilt(False)
            elif key == self.cfg["KEY_OPEN_GRIPPER"]:
                self.bot.gripper.open(wait=False)
            elif key == self.cfg["KEY_CLOSE_GRIPPER"]:
                self.bot.gripper.close(wait=False)
            elif key == KEY_BASE_FORWARD:
                self.move_base(self.base_linear_speed, 0)
            elif key == KEY_BASE_BACKWARD:
                self.move_base(-self.base_linear_speed, 0)
            elif key == KEY_BASE_TURNLEFT:
                self.move_base(0, self.base_angular_speed)
            elif key == KEY_BASE_TURNRIGHT:
                self.move_base(0, -self.base_angular_speed)
            elif key == self.cfg["KEY_RESET"]:
                self.reset()
            else:
                continue
                if key.isdigit():
                    key_int = int(copy.deepcopy(key))
                    if key_int in range(1, 6):
                        self.move_joint_individual(int(key))
                    if key_int in range(6, 10):
                        self.move_joint_individual(int(key) % 5, False)
                    if key_int == 0:
                        self.move_joint_individual(5, False)
                else:
                    print("Pressed invalid key: {}".format(key))
            self.cmd_prev = copy.deepcopy(key)
            self.cmd = None

    def exit(self):
        rospy.loginfo("Exiting...")
        sys.exit(0)

    def signal_handler(self, sig, frame):
        self.exit()


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class KeyboardTeleoperationClient:
    def __init__(self, cfg_file="teleop.yaml"):
        rospy.init_node("keyboard_teleoperation_client", anonymous=True)

        dir_path = os.path.dirname(os.path.realpath(__file__))
        root_path = os.path.dirname(os.path.dirname(dir_path))
        cfg_path = os.path.join(root_path, "config", cfg_file)
        with open(cfg_path, "r") as f:
            self.cfg = yaml.load(f)

        self.pub = rospy.Publisher(self.cfg["ROSTOPIC_TELEOP"], String, queue_size=1)

    def run(self):
        print(HELP_MSG)
        while True:
            cmd = getch()
            if cmd == "\x1b":
                cmd_suffix1 = getch()
                if cmd_suffix1 == "[":
                    cmd_suffix2 = getch()
                    cmd = cmd + cmd_suffix1 + cmd_suffix2

            if cmd == KEY_CTRL_C:
                self.exit()
            else:
                self.pub.publish(cmd)

    def exit(self):
        rospy.loginfo("Exiting...")
        sys.exit(0)

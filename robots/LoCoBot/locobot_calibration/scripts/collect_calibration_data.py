# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
@s-gupta, @amurali and @dgandhi

Run the following to start camera and ar tracker:
roslaunch robot_calibration calibration.launch
"""
import os
import pickle
import rospkg
import signal
import sys

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import numpy as np
import rospy
import tf
import tf.transformations as tfx
import yaml
from absl import app, flags
from pyrobot import Robot
from pyrobot.locobot.camera import LoCoBotCamera

from artag_camera import ARTagCamera

FLAGS = flags.FLAGS
flags.DEFINE_string("data_dir", None, "Directory to store the data for calibration")
flags.DEFINE_integer("num_arm_poses", 5, "If using random poses, number of poses")
flags.DEFINE_integer("num_angles", 5, "Number of pans and tilts.")
flags.DEFINE_string("botname", "locobot", "Name of the bot")

# Parameters for data gathering
MAX_TILT = 0.58
PAN_LIM = np.radians(20)
TILT_LIM = np.radians(30)

ARM_POSES = [
    [0.31, 0.21, -0.178, -0.623, -0.328],
    [-0.5529, 0.4526, -0.3006, -0.7133, -0.0168],
    [-0.0782, 0.251, -0.757, -1.000, -0.0291],
    [0.176, 0.139, -0.096, -0.622, 0.495],
    [-0.392, 1.0, -0.314, -1.181, 0.039],
    [0.457, 0.908, 0.369, -1.273, -0.377],
]


def signal_handler(sig, frame):
    print("Exit")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class CalibrationCamera(LoCoBotCamera, ARTagCamera):
    def __init__(self, configs):
        LoCoBotCamera.__init__(self, configs)
        ARTagCamera.__init__(self, configs)


def mkdir_if_missing(dir_name):
    """
    Makes directory only if it is missing.

    :param dir_name: name of directory
    :type dir_name: string
    """
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)


class CameraCalibration:
    """
    This class sets up arm and camera for data collection for camera
    calibration. It contains code for moving the camera and the arm around,
    recording images, joint angles.
    """

    def __init__(self, botname):
        """
        Constructor, sets up arm and active camera.
        """
        self.bot = Robot(botname)
        self.bot.camera = CalibrationCamera(self.bot.configs)
        self.bot.camera.reset()
        # self.bot.arm.go_home()

        # Initialize listening for transforms
        self.listener = tf.TransformListener()
        rospy.sleep(1)

    def compute_camera_pose(self):
        listener = self.listener
        # Get the transform from the camera links to the AR tag.
        pos1, _ = listener.lookupTransform("head_pan_link", "ar_tag", rospy.Time())
        camera_pan = np.arctan2(pos1[1], pos1[0])
        pos2, _ = listener.lookupTransform("head_tilt_link", "ar_tag", rospy.Time())
        camera_tilt = -np.arctan2(pos2[2], np.linalg.norm(pos2[:2]))
        camera_pan = camera_pan + self.bot.camera.get_pan()
        camera_tilt = camera_tilt + self.bot.camera.get_tilt()
        return camera_pan, camera_tilt

    def move_arm(self, pose):
        """
        Calls appropriate move_it functions to bring arm to desired pose.
        :param pose: numpy array of 5 floats, specifying motor joint angles
        :type pose: numpy array of 5 floats
        :returns: True if move_it succeeded in moving arm to desired pose,
                  otherwise False
        :rtype: boolean
        """
        rospy.loginfo("Planning to pose {}".format(pose))
        result = self.bot.arm.set_joint_positions(pose, plan=False)
        return result

    def get_kinematic_chain(self, target, source, reference):
        """
        Gets the pose for all joints in the kinematic chain from source to
        target.
        :param target: name of target joint
        :param source : name of source joint
        :param reference: name of reference joint as needed by ros function
                          listener.chain
        :type target: string
        :type source: string
        :type reference: string
        :return: kinematic chain, transforms as translation and quaternions,
                 transforms as matrices
        :rtype: list of strings, list of transforms, list of numpy matrices
        """
        # All frames here.
        listener = self.listener
        chain = listener.chain(target, rospy.Time(), source, rospy.Time(), reference)
        Ts, TMs = [], []

        np.set_printoptions(precision=4, suppress=True)
        TMcum = np.eye(4, 4)
        for i in range(len(chain) - 1):
            t = listener.lookupTransform(chain[i + 1], chain[i], rospy.Time())
            t = [[np.float64(_) for _ in t[0]], [np.float64(_) for _ in t[1]]]
            t1_euler = tfx.euler_from_quaternion(t[1])
            tm = tfx.compose_matrix(translate=t[0], angles=t1_euler)
            Ts.append(t)
            TMs.append(tm)

            t = listener.lookupTransform(chain[i + 1], chain[0], rospy.Time())
            t1_euler = tfx.euler_from_quaternion(t[1])
            tm = tfx.compose_matrix(translate=t[0], angles=t1_euler)
            TMcum = np.dot(TMs[i], TMcum)
            eye = np.dot(tm, np.linalg.inv(TMcum))
            assert np.allclose(eye - np.eye(4, 4), np.zeros((4, 4)), atol=0.1)

        # Sanity check to make sure we understand what is happening here.
        # for i in range(len(chain)-1):
        #   t = listener.lookupTransform(chain[i+1], chain[0], rospy.Time(0))
        #   tm = tfx.compose_matrix(translate=t[0], angles=t[1])
        #   TMcum = np.dot(TMs[i], TMcum)
        #   eye = np.dot(tm, np.linalg.inv(TMcum))
        #   print(eye-np.eye(4,4))
        # assert(np.allclose(eye-np.eye(4,4), np.zeros((4,4)), atol=1e-2))
        return chain, Ts, TMs

    def collect_data(self, run_dir):
        """
        Place the arm at a reasonable location, and take multiple images with
        the camera.

        :param run_dir: directory to store collected data
        :type run_dir: string
        """
        mkdir_if_missing(os.path.join(run_dir, "images"))
        states, ar_trans, ar_quat = [], [], []
        ar_img_loc = []
        rgb_tf_tree_trans, rgb_tf_tree_quat, rgb_tf_tree_matrix = [], [], []
        ar_tf_tree_trans, ar_tf_tree_quat, ar_tf_tree_matrix = [], [], []
        arm_pose_ids = []
        i = 0
        N = len(ARM_POSES)
        for arm_pose_id in range(N):
            arm_pose = ARM_POSES[arm_pose_id]
            self.move_arm(arm_pose)

            # Code to compute camera poses.
            camera_pan, camera_tilt = self.compute_camera_pose()

            # Set pans and tilts.
            pans = np.linspace(-PAN_LIM, PAN_LIM, FLAGS.num_angles)
            pans = pans + camera_pan
            pans = np.unique(pans)
            tilts = np.linspace(-TILT_LIM, TILT_LIM, FLAGS.num_angles)
            tilts = tilts + camera_tilt
            tilts = np.minimum(tilts, MAX_TILT)
            tilts = np.unique(tilts)

            for pan in pans:
                # Send pitch to be vertical
                self.bot.camera.set_pan(pan)
                for tilt in tilts:
                    self.bot.camera.set_tilt(tilt)
                    rospy.sleep(1.5)
                    states.append(
                        [self.bot.camera.get_pan(), self.bot.camera.get_tilt()]
                    )
                    img = self.bot.camera.get_rgb()

                    rgb_chain, T, TM = self.get_kinematic_chain(
                        "camera_color_optical_frame", "base_link", "base_link"
                    )
                    assert "camera_link" in rgb_chain
                    T_trans = [t[0] for t in T]
                    T_quat = [t[1] for t in T]
                    rgb_tf_tree_trans.append(T_trans)
                    rgb_tf_tree_quat.append(T_quat)
                    rgb_tf_tree_matrix.append(TM)

                    ar_chain, T, TM = self.get_kinematic_chain(
                        "ar_tag", "base_link", "base_link"
                    )
                    T_trans = [t[0] for t in T]
                    T_quat = [t[1] for t in T]
                    ar_tf_tree_trans.append(T_trans)
                    ar_tf_tree_quat.append(T_quat)
                    ar_tf_tree_matrix.append(TM)

                    position = [np.NaN for _ in range(3)]
                    orientation = [np.NaN for _ in range(4)]
                    ar_pose = self.bot.camera.get_ar_tag_pose()
                    img_pts = [[np.NaN for _ in range(2)] for __ in range(4)]

                    if ar_pose is not None and len(ar_pose.markers) == 1:
                        ar_pose_3d = ar_pose.markers[0].pose.pose
                        position = ar_pose_3d.position
                        position = [getattr(position, x) for x in ["x", "y", "z"]]
                        orientation = ar_pose_3d.orientation
                        orientation = [
                            getattr(orientation, x) for x in ["x", "y", "z", "w"]
                        ]
                        img_pts = []
                        for _ in range(4):
                            pos_img = getattr(
                                ar_pose.markers[0], "pose_img_{:d}".format(_)
                            )
                            pos_img = pos_img.pose.position
                            img_pts.append([pos_img.x, pos_img.y])

                    ar_trans.append(position)
                    ar_img_loc.append(img_pts)
                    ar_quat.append(orientation)
                    file_name = os.path.join(
                        run_dir, "images", "img_{:04d}.png".format(i)
                    )
                    cv2.imwrite(file_name, img[:, :, ::-1])
                    arm_pose_ids.append(arm_pose_id)
                    i = i + 1

            self.save_data(
                run_dir,
                rgb_chain,
                ar_chain,
                states,
                ar_trans,
                ar_quat,
                ar_img_loc,
                rgb_tf_tree_trans,
                rgb_tf_tree_quat,
                rgb_tf_tree_matrix,
                ar_tf_tree_trans,
                ar_tf_tree_quat,
                ar_tf_tree_matrix,
                arm_pose_ids,
            )

    def save_data(
        self,
        run_dir,
        rgb_chain,
        ar_chain,
        states,
        ar_trans,
        ar_quat,
        ar_img_loc,
        rgb_tf_tree_trans,
        rgb_tf_tree_quat,
        rgb_tf_tree_matrix,
        ar_tf_tree_trans,
        ar_tf_tree_quat,
        ar_tf_tree_matrix,
        arm_pose_ids,
    ):
        states = np.array(states)
        ar_trans = np.array(ar_trans)
        ar_quat = np.array(ar_quat)
        ar_img_loc = np.array(ar_img_loc)
        intrinsics = self.bot.camera.get_intrinsics()
        rgb_tf_tree_trans = np.array(rgb_tf_tree_trans)
        rgb_tf_tree_quat = np.array(rgb_tf_tree_quat)
        rgb_tf_tree_matrix = np.array(rgb_tf_tree_matrix)
        ar_tf_tree_trans = np.array(ar_tf_tree_trans)
        ar_tf_tree_quat = np.array(ar_tf_tree_quat)
        ar_tf_tree_matrix = np.array(ar_tf_tree_matrix)
        arm_pose_ids = np.array(arm_pose_ids)

        dt = {
            "rgb_chain": rgb_chain,
            "ar_chain": ar_chain,
            "states": states,
            "ar_trans": ar_trans,
            "ar_quat": ar_quat,
            "ar_img_loc": ar_img_loc,
            "intrinsics": intrinsics,
            "rgb_tf_tree_trans": rgb_tf_tree_trans,
            "rgb_tf_tree_quat": rgb_tf_tree_quat,
            "rgb_tf_tree_matrix": rgb_tf_tree_matrix,
            "ar_tf_tree_trans": ar_tf_tree_trans,
            "ar_tf_tree_quat": ar_tf_tree_quat,
            "ar_tf_tree_matrix": ar_tf_tree_matrix,
            "arm_pose_ids": arm_pose_ids,
        }
        pickle.dump(dt, open(os.path.join(run_dir, "states.pkl"), "wb"))


def main(_):
    mkdir_if_missing(FLAGS.data_dir)
    rospy.loginfo("Storing data in directory: %s", FLAGS.data_dir)
    calibrator = CameraCalibration(FLAGS.botname)
    calibrator.collect_data(FLAGS.data_dir)


if __name__ == "__main__":
    app.run(main)

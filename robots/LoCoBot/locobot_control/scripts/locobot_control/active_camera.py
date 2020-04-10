# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
@amurali, Feb 2018
Simple interface for active head camera
"""
import copy
import threading

import numpy as np
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge
from dynamixel_workbench_msgs.srv import JointCommand
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf import TransformListener

JOINT_STATE_TOPIC = "/joint_states"
JOINT_COMMAND_TOPIC = "/joint_command"

# TODO(s-gupta): Figure out these bounds for the current setup.
MIN_PAN = -2.7
MAX_PAN = 2.6
MIN_TILT = -np.radians(80)
MAX_TILT = np.radians(100)
RESET_PAN = 0.0
RESET_TILT = 0.0
BB_SIZE = 5

ROSTOPIC_CAMERA_INFO_STREAM = "/camera/color/camera_info"
ROSTOPIC_CAMERA_IMAGE_STREAM = "/camera/color/image_raw"
ROSTOPIC_CAMERA_DEPTH_STREAM = "/camera/depth/image_rect_raw"
ROSTOPIC_ALIGNED_CAMERA_DEPTH_STREAM = "/camera/aligned_depth_to_color/image_raw"
ROSTOPIC_AR_POSE_MARKER = "/ar_pose_marker"
MAX_DEPTH = 3.0
BASE_FRAME = "arm_base_link"
KINECT_FRAME = "camera_color_optical_frame"

ROSTOPIC_SET_PAN = "/pan/command"
ROSTOPIC_SET_TILT = "/tilt/command"


def constrain_within_range(value, MIN, MAX):
    return min(max(value, MIN), MAX)


def is_within_range(value, MIN, MAX):
    return (value <= MAX) and (value >= MIN)


class ActiveCamera(object):
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.camera_info_lock = threading.RLock()
        self.ar_tag_lock = threading.RLock()

        # Setup to control camera.
        self.joint_cmd_srv = rospy.ServiceProxy(JOINT_COMMAND_TOPIC, JointCommand)

        # Subscribe to camera pose and instrinsic streams.
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._camera_pose_callback)
        rospy.Subscriber(
            ROSTOPIC_CAMERA_INFO_STREAM, CameraInfo, self.camera_info_callback
        )
        self.img = None
        rospy.Subscriber(
            ROSTOPIC_CAMERA_IMAGE_STREAM,
            Image,
            lambda x: self.img_callback(x, "img", "bgr8"),
        )
        self.depth = None
        rospy.Subscriber(
            ROSTOPIC_CAMERA_DEPTH_STREAM,
            Image,
            lambda x: self.img_callback(x, "depth", None),
        )
        self.depth_registered = None
        rospy.Subscriber(
            ROSTOPIC_ALIGNED_CAMERA_DEPTH_STREAM,
            Image,
            lambda x: self.img_callback(x, "depth_registered", None),
        )
        rospy.Subscriber(ROSTOPIC_AR_POSE_MARKER, AlvarMarkers, self.alvar_callback)
        self.img = None
        self.ar_tag_pose = None

        self._transform_listener = TransformListener()
        self._update_camera_extrinsic = True
        self.camera_extrinsic_mat = None
        self.set_pan_pub = rospy.Publisher(ROSTOPIC_SET_PAN, Float64, queue_size=1)
        self.set_tilt_pub = rospy.Publisher(ROSTOPIC_SET_TILT, Float64, queue_size=1)

    def _camera_pose_callback(self, msg):
        pan_id = msg.name.index("head_pan_joint")
        tilt_id = msg.name.index("head_tilt_joint")
        self.pan = msg.position[pan_id]
        self.tilt = msg.position[tilt_id]

    def camera_info_callback(self, msg):
        self.camera_info_lock.acquire()
        self.camera_info = msg
        self.camera_P = np.array(msg.P).reshape((3, 4))
        self.camera_info_lock.release()

    def alvar_callback(self, msg):
        self.ar_tag_lock.acquire()
        self.ar_tag_pose = msg
        self.ar_tag_lock.release()

    @property
    def state(self):
        return self.get_state()

    def get_state(self):
        return [self.pan, self.tilt]

    def get_pan(self):
        return self.pan

    def get_tilt(self):
        return self.tilt

    def set_pan(self, pan, wait=True):
        pan = constrain_within_range(
            np.mod(pan + np.pi, 2 * np.pi) - np.pi, MIN_PAN, MAX_PAN
        )
        # self.joint_cmd_srv('rad', 8, pan)
        self.set_pan_pub.publish(pan)
        if wait:
            rospy.sleep(3)

    def set_tilt(self, tilt, wait=True):
        tilt = constrain_within_range(
            np.mod(tilt + np.pi, 2 * np.pi) - np.pi, MIN_TILT, MAX_TILT
        )
        # self.joint_cmd_srv('rad', 9, tilt)
        self.set_tilt_pub.publish(tilt)
        if wait:
            rospy.sleep(3)

    def reset(self):
        self.set_pan(RESET_PAN)
        self.set_tilt(RESET_TILT)

    def img_callback(self, msg, field, typ=None):
        if typ is None:
            setattr(self, field, self.cv_bridge.imgmsg_to_cv2(msg))
        else:
            setattr(
                self, field, self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding=typ)
            )

    def get_ar_tag_pose(self):
        self.ar_tag_lock.acquire()
        ar_tag_pose = copy.deepcopy(self.ar_tag_pose)
        self.ar_tag_lock.release()
        return ar_tag_pose

    def get_image(self):
        if self.img is not None:
            return self.img * 1
        else:
            raise RuntimeError("No image found. Please check the camera")

    def get_depth(self):
        if self.depth_registered is not None:
            return self.depth_registered * 1
        elif self.depth is not None:
            return self.depth * 1
        else:
            raise RuntimeError("No depth found. Please check the camera")

    def get_intrinsics(self):
        self.camera_info_lock.acquire()
        P = np.array(self.camera_info.P).copy()
        self.camera_info_lock.release()
        return P.reshape((3, 4))

    def _process_depth(self, cur_depth=None):
        if cur_depth is None:
            cur_depth = self.get_depth()
        cur_depth = cur_depth / 1000.0  # conversion from mm to m
        cur_depth[cur_depth > MAX_DEPTH] = 0.0
        return cur_depth

    def _get_z_mean(self, depth, pt, bb=BB_SIZE):
        sum_z = 0.0
        nps = 0
        for i in range(bb * 2):
            for j in range(bb * 2):
                new_pt = [pt[0] - bb + i, pt[1] - bb + j]
                try:
                    new_z = depth[int(new_pt[0]), int(new_pt[1])]
                    if new_z > 0.0:
                        sum_z += new_z
                        nps += 1
                except:
                    pass
        if nps == 0.0:
            return 0.0
        else:
            return sum_z / nps

    def _get_3D_camera(self, pt, norm_z=None):
        assert len(pt) == 2
        cur_depth = self._process_depth()
        z = self._get_z_mean(cur_depth, [pt[0], pt[1]])
        if z == 0.0:
            raise RuntimeError
        if norm_z is not None:
            z = z / norm_z
        u = pt[1]
        v = pt[0]
        P = copy.deepcopy(self.camera_P)
        P_n = np.zeros((3, 3))
        P_n[:, :2] = P[:, :2]
        P_n[:, 2] = P[:, 3] + P[:, 2] * z
        P_n_inv = np.linalg.inv(P_n)
        temp_p = np.dot(P_n_inv, np.array([u, v, 1]))
        temp_p = temp_p / temp_p[-1]
        temp_p[-1] = z
        return temp_p

    def _convert_frames(self, pt):
        assert len(pt) == 3
        ps = PointStamped()
        ps.header.frame_id = KINECT_FRAME
        ps.point.x, ps.point.y, ps.point.z = pt
        base_ps = self._transform_listener.transformPoint(BASE_FRAME, ps)
        base_pt = np.array([base_ps.point.x, base_ps.point.y, base_ps.point.z])
        return base_pt

    def get_3D(self, pt, z_norm=None):
        temp_p = self._get_3D_camera(pt, z_norm)
        base_pt = self._convert_frames(temp_p)
        return base_pt


if __name__ == "__main__":
    rospy.init_node("active_camera", anonymous=True)
    ac = ActiveCamera()
    rospy.spin()

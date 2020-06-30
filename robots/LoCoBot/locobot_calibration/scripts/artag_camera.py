# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import threading
import rospy
import copy
from ar_track_alvar_msgs.msg import AlvarMarkers

ROSTOPIC_AR_POSE_MARKER = "/ar_pose_marker"


class ARTagCamera(object):
    def __init__(self, configs):
        self.ar_tag_pose = None
        self.ar_tag_lock = threading.RLock()
        rospy.Subscriber(ROSTOPIC_AR_POSE_MARKER, AlvarMarkers, self.alvar_callback)

    def alvar_callback(self, msg):
        self.ar_tag_lock.acquire()
        self.ar_tag_pose = msg
        self.ar_tag_lock.release()

    def get_ar_tag_pose(self):
        self.ar_tag_lock.acquire()
        ar_tag_pose = copy.deepcopy(self.ar_tag_pose)
        self.ar_tag_lock.release()
        return ar_tag_pose

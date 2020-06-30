#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Publishes the calibrated camera and other misc transforms
"""

import sys
import os
import json
import rospy
import tf

DEFAULT_CONFIG_FNAME = "default.json"
CALIBRATED_CONFIG_FNAME = "calibrated.json"


def get_abs_path(file_name):
    return os.path.join(os.environ["HOME"], ".robot", file_name)


class CalibrationTransformPublisher(object):
    """
    Publishes the calibrated camera and other misc transforms
    """

    def __init__(self, file_name=None):
        """
        Note that file_name is not absolute path. All calibration files are
        assumed to be in ~/.robot/
        """
        params = {}
        if not file_name:
            if os.path.exists(get_abs_path(CALIBRATED_CONFIG_FNAME)):
                file_name = CALIBRATED_CONFIG_FNAME
            else:
                rospy.logerr("Using default camera calibration")
                rospy.logerr("For better performance, calibrate your system.")
                file_name = DEFAULT_CONFIG_FNAME
        file_path = get_abs_path(file_name)
        if os.path.exists(file_path):
            rospy.loginfo("Loading transforms from {:}".format(file_path))
            with open(file_path, "r") as f:
                params = json.load(f)
            self.params = params
            rospy.logwarn("Will publish: ")
            for t in self.params.values():
                rospy.logwarn("  {} to {}".format(t["from"], t["to"]))
            self.br = tf.TransformBroadcaster()
            self.publish_transforms()
        else:
            rospy.logerr("Unable to find calibration config file {}".format(file_path))
            sys.exit(0)

    def publish_transforms(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for t in self.params.values():
                self.br.sendTransform(
                    t["trans"], t["quat"], rospy.Time.now(), t["to"], t["from"]
                )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("calibration_tf_broadcaster")
    ctp = CalibrationTransformPublisher()

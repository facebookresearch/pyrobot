# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import argparse
import signal
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from pyrobot import Robot
from sklearn.cluster import DBSCAN


def signal_handler(sig, frame):
    print("Exit")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def filter_points(pts, colors, z_lowest=0.01):
    valid = pts[:, 2] > z_lowest
    valid = np.logical_and(valid, pts[:, 0] < 0.5)
    valid = np.logical_and(valid, pts[:, 1] < 0.4)
    valid = np.logical_and(valid, pts[:, 1] > -0.4)
    pts = pts[valid]
    colors = colors[valid]
    return pts, colors


def segment_objects(pts):
    db = DBSCAN(eps=0.02, min_samples=15).fit(pts)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_
    return labels, core_samples_mask


def draw_segments(pts, labels, core_samples_mask):
    num_threshold = 400
    plt.clf()
    plt.scatter(pts[:, 0], pts[:, 1])
    plt.xlim(-0.4, 0.4)
    plt.ylim(0, 0.5)
    plt.xlabel("Y axis of the base link")
    plt.ylabel("X axis of the base link")
    plt.savefig("raw_pts.png")
    plt.clf()
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    unique_labels = set(labels)
    useful_labels = []
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    print("Estimated number of clusters: %d" % n_clusters_)
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]
            continue
        class_member_mask = labels == k

        xy = pts[class_member_mask & core_samples_mask]
        print("Label:[%d]   # of pts:%d" % (k, xy.shape[0]))
        if xy.shape[0] > num_threshold:
            useful_labels.append(k)
        plt.plot(xy[:, 0], xy[:, 1], "o", markerfacecolor=tuple(col), markersize=1)
        plt.xlim(-0.4, 0.4)
        plt.ylim(0, 0.5)
        plt.xlabel("Y axis of the base link")
        plt.ylabel("X axis of the base link")
    plt.savefig("seg_pts.png")
    print("Number of clusters after filtering:", len(useful_labels))
    return useful_labels


def sample_pt(pts, labels, useful_labelset, gripper_length=0.13):
    tgt_label = np.random.choice(useful_labelset, 1)[0]
    tgt_pts = pts[labels == tgt_label]
    center = np.mean(tgt_pts, axis=0)
    bbox_xy_min = np.amin(tgt_pts[:, :2], axis=0) - 0.03
    bbox_xy_max = np.amax(tgt_pts[:, :2], axis=0) + 0.03
    bbox_xy = np.stack((bbox_xy_min, bbox_xy_max), axis=0)
    tgt_edge = np.random.choice(2, 1)[0]
    if tgt_edge == 0:
        tgt_x = np.random.uniform(bbox_xy[0, 0], bbox_xy[1, 0], 1)[0]
        tgt_y = bbox_xy[int(np.random.choice(2, 1)[0]), 1]
    else:
        tgt_y = np.random.uniform(bbox_xy[0, 1], bbox_xy[1, 1], 1)[0]
        tgt_x = bbox_xy[int(np.random.choice(2, 1)[0]), 0]
    tgt_z = max(center[2], 0)
    tgt_z += gripper_length
    center[2] = tgt_z
    mid_pt = np.array([tgt_x, tgt_y, tgt_z])

    start_pt = np.array([tgt_x, tgt_y, 0.35])
    return start_pt, mid_pt, center


def push(bot, z_lowest=0.01, gripper_length=0.10):
    ee_pose = bot.arm.pose_ee
    if ee_pose[0][2] < 0.20:
        bot.arm.go_home()
    bot.arm.set_joint_positions([1.96, 0.52, -0.51, 1.67, 0.01], plan=False)
    pts, colors = bot.camera.get_current_pcd(in_cam=False)
    pts, colors = filter_points(pts, colors, z_lowest=z_lowest)
    X = pts[:, :2]
    labels, core_samples_mask = segment_objects(X)
    useful_labelset = draw_segments(X, labels, core_samples_mask)
    start_pt, mid_pt, center = sample_pt(
        pts, labels, useful_labelset, gripper_length=gripper_length
    )
    print("Going to: ", start_pt.tolist())
    result = bot.arm.set_ee_pose_pitch_roll(
        position=start_pt, pitch=np.pi / 2, roll=0, plan=False, numerical=False
    )
    if not result:
        return
    down_disp = mid_pt - start_pt
    bot.arm.move_ee_xyz(down_disp, plan=False, numerical=False)
    hor_disp = 2 * (center - mid_pt)
    bot.arm.move_ee_xyz(hor_disp, plan=False, numerical=False)


def main():
    parser = argparse.ArgumentParser(description="Argument Parser")
    parser.add_argument(
        "--floor_height", type=float, default=0.03, help="the z coordinate of the floor"
    )
    parser.add_argument(
        "--gripper_len",
        type=float,
        default=0.12,
        help="the gripper length (length from gripper_link" "to the tip of the gripper",
    )
    args = parser.parse_args()

    np.set_printoptions(precision=4, suppress=True)
    bot = Robot("locobot", use_arm=True, use_base=False, use_gripper=True)
    bot.gripper.close()
    bot.camera.set_pan_tilt(0, 0.7, wait=True)
    bot.arm.go_home()
    while True:
        try:
            push(bot, z_lowest=args.floor_height, gripper_length=args.gripper_len)
        except:
            pass
        time.sleep(1)


if __name__ == "__main__":
    main()

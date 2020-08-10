# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import signal
import sys
import time

import numpy as np
import open3d
from pyrobot import Robot


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


def main():
    parser = argparse.ArgumentParser(description="Argument Parser")
    parser.add_argument(
        "--floor_height", type=float, default=0.03, help="the z coordinate of the floor"
    )
    args = parser.parse_args()
    np.set_printoptions(precision=4, suppress=True)
    bot = Robot("locobot")
    bot.camera.set_pan_tilt(0, 0.7, wait=True)
    bot.arm.go_home()
    bot.arm.set_joint_positions([1.96, 0.52, -0.51, 1.67, 0.01], plan=False)
    vis = open3d.Visualizer()
    vis.create_window("3D Map")
    pcd = open3d.PointCloud()
    coord = open3d.create_mesh_coordinate_frame(1, [0, 0, 0])
    vis.add_geometry(pcd)
    vis.add_geometry(coord)
    while True:
        pts, colors = bot.camera.get_current_pcd(in_cam=False)
        pts, colors = filter_points(pts, colors, z_lowest=args.floor_height)
        pcd.clear()
        # note that open3d.Vector3dVector is slow
        pcd.points = open3d.Vector3dVector(pts)
        pcd.colors = open3d.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)


if __name__ == "__main__":
    main()

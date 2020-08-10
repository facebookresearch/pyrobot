# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import signal
import sys
import time

import open3d
from pyrobot import Robot


def signal_handler(sig, frame):
    print("Exit")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def main():
    bot = Robot("kinect2")
    vis = open3d.Visualizer()
    vis.create_window("3D Map")
    pcd = open3d.PointCloud()
    coord = open3d.create_mesh_coordinate_frame(1, [0, 0, 0])
    vis.add_geometry(pcd)
    vis.add_geometry(coord)
    # We update the viewer every a few seconds
    update_time = 4
    while True:
        start_time = time.time()
        pts, colors = bot.camera.get_current_pcd()
        pcd.clear()
        # note that open3d.Vector3dVector is slow
        pcd.points = open3d.Vector3dVector(pts)
        pcd.colors = open3d.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        s_t = time.time()
        while time.time() - s_t < update_time:
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.1)
        end_time = time.time()
        process_time = end_time - start_time
        print("Updating every {0:.2f}s".format(process_time))


if __name__ == "__main__":
    main()

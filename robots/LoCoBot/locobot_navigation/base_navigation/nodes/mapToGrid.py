#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from pyrobot import Robot

# params to be given

CELL_RESOLUTION = 0.02

MAP_TOPIC = "/occupancy_map"


class PcdToOcGrid:
    def __init__(self, z_lower_treshold, z_upper_treshold, x_min, y_min, x_max, y_max):

        self.bot = Robot(
            "locobot", base_config={"build_map": True, "base_planner": "none"}
        )

        self.ocGrid = None  #
        self.points = None  # these are the points from orbslam
        self.colors = None

        self.z_lower_treshold = z_lower_treshold
        self.z_upper_treshold = z_upper_treshold

        # occupancy grid params
        self.xMax = None
        self.xMin = None
        self.yMin = None
        self.yMax = None

        # grid
        self.grid = OccupancyGrid()
        self.xCells = None
        self.yCells = None

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.initGrid()

    def populatePoints(self):
        self.points, self.colors = self.bot.base.base_state.vslam.get_3d_map()

    def populateMap(self):
        if self.points is None:
            return

        self.xCells = int(np.ceil((self.xMax - self.xMin) / CELL_RESOLUTION))
        self.yCells = int(np.ceil((self.yMax - self.yMin) / CELL_RESOLUTION))
        self.ocGrid = np.zeros(self.xCells * self.yCells)

        good_point_indxs = np.logical_and(
            self.points[:, 2] >= self.z_lower_treshold,
            self.points[:, 2] <= self.z_upper_treshold,
        )

        self.points = self.points[
            good_point_indxs, :2
        ]  # z tresholding and removing z column

        map_origin = np.array([self.xMin, self.yMin])
        self.points = self.points[:, :2] - map_origin
        self.points = np.floor(self.points / CELL_RESOLUTION).astype(int)
        self.points = (
            self.points[:, 1] * self.xCells + self.points[:, 0]
        )  # at this stage they are indices
        self.ocGrid[self.points] = 100

    def calcSize(self):
        if self.points is None:
            self.xMin = self.x_min
            self.xMax = self.x_max
            self.yMin = self.y_min
            self.yMax = self.y_max
            return

        self.num_points = self.points.shape

        mins = np.amin(self.points, axis=0)
        maxs = np.amax(self.points, axis=0)

        self.xMax = max(maxs[0], self.x_max)
        self.yMin = min(mins[1], self.y_min)
        self.xMin = min(mins[0], self.x_min)
        self.yMax = max(maxs[1], self.y_max)

    def initGrid(self):
        self.grid.header.seq = 1
        self.grid.header.frame_id = "/map"
        self.grid.info.origin.position.z = 0
        self.grid.info.origin.orientation.x = 0
        self.grid.info.origin.orientation.y = 0
        self.grid.info.origin.orientation.z = 0
        self.grid.info.origin.orientation.w = 1

    def updateGrid(self):
        if self.points is None:
            return
        self.grid.header.seq = self.grid.header.seq + 1
        self.grid.header.stamp = rospy.Time.now()
        self.grid.info.map_load_time = rospy.Time.now()
        self.grid.info.resolution = CELL_RESOLUTION
        self.grid.info.width = self.xCells
        self.grid.info.height = self.yCells
        self.grid.info.origin.position.x = self.xMin
        self.grid.info.origin.position.y = self.yMin
        self.grid.data = self.ocGrid


def main():
    convertor = PcdToOcGrid(0.3, 0.5, -2.5, -2.5, 2.5, 2.5)
    ocGrid_publisher = rospy.Publisher(MAP_TOPIC, OccupancyGrid, queue_size=1)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        convertor.populatePoints()
        convertor.calcSize()
        convertor.populateMap()
        convertor.updateGrid()
        ocGrid_publisher.publish(convertor.grid)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

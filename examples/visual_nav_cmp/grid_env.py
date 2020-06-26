# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np


def generate_goal_images(
    map_scales, map_crop_sizes, n_ori, goal_dist, goal_theta, rel_goal_orientation
):
    goal_dist = goal_dist[:, 0]
    goal_theta = goal_theta[:, 0]
    rel_goal_orientation = rel_goal_orientation[:, 0]

    goals = []
    # Generate the map images.
    for i, (sc, map_crop_size) in enumerate(zip(map_scales, map_crop_sizes)):
        goal_i = np.zeros(
            (goal_dist.shape[0], map_crop_size, map_crop_size, n_ori), dtype=np.float32
        )
        x = goal_dist * np.cos(goal_theta) * sc + (map_crop_size - 1.0) / 2.0
        y = goal_dist * np.sin(goal_theta) * sc + (map_crop_size - 1.0) / 2.0

        for j in range(goal_dist.shape[0]):
            gc = rel_goal_orientation[j]
            x0 = np.floor(x[j]).astype(np.int32)
            x1 = x0 + 1
            y0 = np.floor(y[j]).astype(np.int32)
            y1 = y0 + 1
            if x0 >= 0 and x0 <= map_crop_size - 1:
                if y0 >= 0 and y0 <= map_crop_size - 1:
                    goal_i[j, y0, x0, gc] = (x1 - x[j]) * (y1 - y[j])
                if y1 >= 0 and y1 <= map_crop_size - 1:
                    goal_i[j, y1, x0, gc] = (x1 - x[j]) * (y[j] - y0)

            if x1 >= 0 and x1 <= map_crop_size - 1:
                if y0 >= 0 and y0 <= map_crop_size - 1:
                    goal_i[j, y0, x1, gc] = (x[j] - x0) * (y1 - y[j])
                if y1 >= 0 and y1 <= map_crop_size - 1:
                    goal_i[j, y1, x1, gc] = (x[j] - x0) * (y[j] - y0)

        goals.append(goal_i)
    return goals


def image_pre(images, modalities):
    # Assumes images are ...xHxWxC
    images[..., :3] = images[..., :3] - 128
    return images


class GridEnv(object):
    """
    Class that maintains and generates relative goal position in a grid world.
    This class is used to provide the relative goal position as an image to the
    policy.
    """

    def __init__(self):
        # Goal and state are maintained in cm.
        self.theta_unit = np.pi / 2.0
        self.xy_unit = 5.0
        None

    def _get_relative_goal_loc(self, goal_loc, loc, theta):
        r = np.sqrt(np.sum(np.square(goal_loc - loc), axis=1))
        t = np.arctan2(goal_loc[:, 1] - loc[:, 1], goal_loc[:, 0] - loc[:, 0])
        t = t - theta[:, 0] + np.pi / 2
        return np.expand_dims(r, axis=1), np.expand_dims(t, axis=1)

    def _get_loc_axis(self, node):
        """Based on the node orientation returns X, and Y axis. Used to sample the
        map in egocentric coordinate frame.
        """
        x, y, t = node[:, [0]], node[:, [1]], node[:, [2]]
        theta = t * self.theta_unit
        loc = np.concatenate((x, y), axis=1)
        x_axis = np.concatenate((np.cos(theta), np.sin(theta)), axis=1)
        thetay = theta + np.pi / 2.0
        y_axis = np.concatenate((np.cos(thetay), np.sin(thetay)), axis=1)
        return loc, x_axis, y_axis, theta

    def _transform_state(self, state):
        t_state = state.copy()
        t_state[:, :2] = t_state[:, :2] * self.xy_unit
        t_state[:, 2] = t_state[:, 2] * self.theta_unit
        return t_state

    def reset(self, goal):
        """Reset internal states, and set goal location.

        :param goal: Specified relative offset. List containing [x, y, theta]
        :type goal: list
        """
        self.goal = np.array([goal], dtype=np.float64)
        self.goal[:, :2] = self.goal[:, :2] / self.xy_unit
        self.goal[:, 2] = self.goal[:, 2] / self.theta_unit
        self.state = np.array([[0, 0, 1]], dtype=np.float64)
        self.previous_state = self.state.copy()
        return self._transform_state(self.state)

    def get_features(self):
        out1 = self._get_goal_images()
        out2 = self._get_incremental_pose()
        out1.update(out2)
        return out1

    def pre_features(self, inputs):
        inputs["imgs"] = image_pre(inputs["imgs"], "rgb")
        return inputs

    def take_action(self, action):
        step_size = 8
        self.previous_state = self.state.copy()
        if action == 0:
            None
        elif action == 1:
            self.state[0, 2] = np.mod(self.state[0, 2] - 1, 4)
        elif action == 2:
            self.state[0, 2] = np.mod(self.state[0, 2] + 1, 4)
        elif action == 3:
            orientation = self.state[0, 2]
            if orientation == 0:
                self.state[0, 0] += step_size
            elif orientation == 1:
                self.state[0, 1] += step_size
            elif orientation == 2:
                self.state[0, 0] -= step_size
            elif orientation == 3:
                self.state[0, 1] -= step_size
        return self._transform_state(self.state)

    def _get_goal_images(self):
        loc, x_axis, y_axis, theta = self._get_loc_axis(self.state)
        goal_loc, _, _, _ = self._get_loc_axis(self.goal)
        rel_goal_orientation = np.mod(np.int32(self.state[:, 2:] - self.goal[:, 2:]), 4)
        goal_dist, goal_theta = self._get_relative_goal_loc(goal_loc, loc, theta)

        n_ori = 4
        map_crop_sizes = [16 for __ in range(3)]
        map_scales = [np.power(2.0, i - 5) for i in range(3)]

        goals = generate_goal_images(
            map_scales,
            map_crop_sizes,
            n_ori,
            goal_dist,
            goal_theta,
            rel_goal_orientation,
        )
        outs = {}
        for i in range(len(map_scales)):
            outs["ego_goal_imgs_{:d}".format(i)] = np.expand_dims(goals[i], axis=1)
        return outs

    def _get_incremental_pose(self):
        loc, _, _, theta = self._get_loc_axis(self.state)
        previous_loc, _, _, previous_theta = self._get_loc_axis(self.previous_state)
        incremental_locs_ = np.reshape(loc - previous_loc, [1, 1, -1])
        t = -np.pi / 2 + np.reshape(theta * 1, [1, 1, -1])
        incremental_locs = incremental_locs_ * 1
        cossin = np.concatenate((np.cos(t), np.sin(t)), axis=-1)
        incremental_locs[:, :, 0] = np.sum(incremental_locs_ * cossin, axis=-1)
        cossin = np.concatenate([np.cos(t + np.pi / 2), np.sin(t + np.pi / 2)], axis=-1)
        incremental_locs[:, :, 1] = np.sum(incremental_locs_ * cossin, axis=-1)
        incremental_thetas = np.reshape(theta - previous_theta, [1, 1, -1])
        outs = {}
        outs["incremental_locs"] = incremental_locs
        outs["incremental_thetas"] = incremental_thetas
        return outs

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import pytest
import time

import numpy as np
import matplotlib as mpl

if os.environ.get("DISPLAY", "") == "":
    print("no display found. Using non-interactive Agg backend")
    mpl.use("Agg")
from matplotlib import gridspec
from matplotlib import pyplot as plt
from pyrobot.locobot.base_control_utils import position_control_init_fn
from pyrobot.locobot.bicycle_model import wrap_theta


def _get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


def _mkdir_if_missing(dir_name):
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)


def _states_close(state1, state2):
    dist = np.linalg.norm(state1[:2] - state2[:2])
    angle = wrap_theta(state1[2] - state2[2])
    assert dist < 0.05
    assert np.abs(angle) < 10 / 180.0 * np.pi


def _plot_test(states, start_pos, goal_pos, file_name):
    gs = gridspec.GridSpec(3, 6)
    gs.update(left=0.05, right=0.95, hspace=0.05, wspace=0.10)
    axes = [plt.subplot(gs[i, :3]) for i in range(3)]
    axes.append(plt.subplot(gs[:, 3:]))
    ax = axes[-1]
    r = 0.1
    for state in states[::10, :]:
        x, y, theta = state
        x_axis = [r * np.cos(theta), r * np.sin(theta)]
        y_axis = [r * np.cos(theta + np.pi / 2), r * np.sin(theta + np.pi / 2)]
        ax.plot([x, x + x_axis[0]], [y, y + x_axis[1]], "r", lw=1)
        ax.plot([x, x + y_axis[0]], [y, y + y_axis[1]], "g", lw=1)
    if states.shape[0] > 0:
        ax.plot(states[0, 0], states[0, 1], "bs")
        ax.plot(states[-1, 0], states[-1, 1], "b*")
    ax.axis("equal")
    ax.grid(True)
    start_str = ("{:0.1f}, " * 3).format(*start_pos)
    end_str = ("{:0.1f}, " * 3).format(*goal_pos)
    ax.set_title(start_str + " to " + end_str)

    labels = ["x", "y", "theta"]
    for i, x in enumerate(states.T):
        axes[i].plot(x)
        axes[i].set_title(labels[i], x=0.1, y=0.8)
        axes[i].grid(True)
    if file_name is not None:
        plt.savefig(file_name, bbox_inches="tight")
    plt.close()


def _test_init(init_type, start_pos, goal_pos, reverse):
    dt = 0.1
    max_v = 0.2
    max_w = 0.4
    init_states = position_control_init_fn(
        init_type, start_pos, goal_pos, dt, max_v, max_w, reverse
    )

    file_name = "position_{:s}_{:s}_reverse{:d}-{:s}.pdf"
    goal_pos_str = ["{:0.1f}".format(x) for x in goal_pos]
    file_name = file_name.format(
        ",".join(goal_pos_str), init_type, int(reverse), _get_time_str()
    )
    tmp_dir = os.path.join(os.path.dirname(__file__), "tmp")
    _mkdir_if_missing(tmp_dir)
    file_name = os.path.join(tmp_dir, file_name)

    _plot_test(init_states, start_pos, goal_pos, file_name)
    _states_close(init_states[-1, :], goal_pos)


goal_poss = np.array(
    [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, -0.2, 0.0],
        [1.0, 1.0, 0.0],
        [-1.0, -1.0, 0],
        [0.0, 0.0, np.pi / 2.0],
        [1.0, 1.0, 0.0],
        [-1.0, 1.0, 0],
        [0.0, 1.0, np.pi / 2.0],
        [0.1, 0.1, 0.0],
        [-0.1, -0.1, 0],
        [0.0, 0.0, 0.1],
    ],
    dtype=np.float32,
)


@pytest.mark.parametrize("init_type", ["sharp", "smooth"])
@pytest.mark.parametrize("reverse", [False, True])
@pytest.mark.parametrize("goal_pos", goal_poss)
def test_init(init_type, reverse, goal_pos):
    start_pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    _test_init(init_type, start_pos, goal_pos, reverse=reverse)

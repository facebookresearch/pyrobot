# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import pytest

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import numpy as np

from cmp_runner import CMPRunner


@pytest.mark.parametrize("test_dir", ["test-data/v0/"])
def test_cmp_tfcode(test_dir):
    action_strings = {0: "stay", 1: "right", 2: "left", 3: "forward"}
    cmp_runner = CMPRunner("./model.ckpt-120003")
    tt = os.listdir(test_dir)
    tt = filter(lambda x: "jpg" in x, tt)
    tt.sort()
    goal = np.loadtxt(os.path.join(test_dir, "goal.txt"))
    cmp_runner.set_new_goal(goal.tolist())

    for i in range(len(tt)):
        gt_action = tt[i].split("_")[1].split(".")[0]
        img = cv2.imread(os.path.join(test_dir, tt[i]))
        img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
        img = img[np.newaxis, np.newaxis, np.newaxis, :, :, ::-1]
        img = img.astype(np.float32)
        model_action, next_state = cmp_runner.compute_action(img)
        model_action = model_action[0]
        next_state = next_state[0]
        assert action_strings[model_action] == gt_action


if __name__ == "__main__":
    test_cmp_tfcode("test-data/v0")

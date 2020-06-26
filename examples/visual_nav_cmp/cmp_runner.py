# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import tensorflow as tf

from cmp_policy import CMPPolicy
from config import get_cfg_defaults
from grid_env import GridEnv
from tf_utils import prepare_feed_dict


class CMPRunner(object):
    def __init__(self, model_path):
        config = self._get_tf_config()
        args = get_cfg_defaults()

        self.tf_graph = tf.Graph()

        with self.tf_graph.as_default():
            self.policy = CMPPolicy(
                args,
                is_training=False,
                batch_norm_is_training=False,
                only_single_step_graph=True,
            )

            self.sess = tf.Session(config=config)
            self.sess.run(self.policy.init_op)
            self.policy.saver_op.restore(self.sess, model_path)

    def set_new_goal(self, goal):
        goal_raw = np.array(goal)
        self.grid_env = GridEnv()
        _ = self.grid_env.reset(goal_raw)
        self.state_features, self.step_data_cache = [], []

        feed_dict = {}
        self.net_state = self.sess.run(
            self.policy.train_ops["init_state"], feed_dict=feed_dict
        )
        self.net_state = dict(zip(self.policy.train_ops["state_names"], self.net_state))

    def compute_action(self, image):
        f = self.grid_env.get_features()
        f["imgs"] = image.copy()
        f = self.grid_env.pre_features(f)
        f.update(self.net_state)

        self.state_features.append(f)
        feed_dict = prepare_feed_dict(
            self.policy.input_tensors["step"], self.state_features[-1]
        )
        feed_dict[self.policy.train_ops["batch_norm_is_training_op"]] = False

        kwargs = {}
        outs = self.sess.run(
            [
                self.policy.train_ops["step"],
                self.policy.train_ops["step_data_cache"],
                self.policy.train_ops["updated_state"],
            ],
            feed_dict=feed_dict,
            **kwargs
        )
        action_probs = np.expand_dims(outs[0], axis=1)
        action = [np.argmax(action_probs[0, 0, :])]
        self.step_data_cache.append(
            dict(zip(self.policy.train_ops["step_data_cache_names"], outs[1]))
        )
        self.net_state = outs[2]

        assert action_probs.shape[1] == 1
        next_state = self.grid_env.take_action(action[0])
        self.net_state = dict(zip(self.policy.train_ops["state_names"], self.net_state))
        return action, next_state

    def _get_tf_config(self):
        config = tf.ConfigProto()
        config.device_count["GPU"] = 1
        config.gpu_options.allow_growth = True
        config.intra_op_parallelism_threads = 0
        config.inter_op_parallelism_threads = 0
        return config

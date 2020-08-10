# Copyright 2016 The TensorFlow Authors All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Code for setting up the network for CMP.

Sets up the mapper and the planner.
"""

import numpy as np
import tensorflow as tf

from cmp_utils import fr_v2, get_map_from_images, running_combine
from cmp_utils import value_iteration_network
from tf_utils import setup_inputs, fc_network


class CMPPolicy(object):
    def __init__(
        self, args, is_training, batch_norm_is_training, only_single_step_graph
    ):
        self.input_tensors = {}
        self.train_ops = {}
        # Set up the model.
        tf.set_random_seed(args.solver.seed)
        task_params = args.navtask.task_params

        batch_norm_is_training_op_ph = tf.placeholder_with_default(
            batch_norm_is_training, shape=[], name="batch_norm_is_training_op"
        )
        batch_norm_is_training_op = tf.identity(batch_norm_is_training_op_ph)

        # Setup the inputs
        self.input_tensors["step"] = self.setup_inputs(
            task_params.batch_size,
            task_params.img_height,
            task_params.img_width,
            task_params.img_channels,
            task_params.map_crop_sizes,
            task_params.map_channels,
            task_params.goal_channels,
        )
        self.init_fn = None

        self.vision_ops = get_map_from_images(
            self.input_tensors["step"]["imgs"],
            args.mapper_arch,
            task_params,
            args.solver.freeze_conv,
            args.solver.wt_decay,
            is_training,
            batch_norm_is_training_op,
            num_maps=len(task_params.map_crop_sizes),
        )

        # Set up caching of vision features if needed.
        if args.solver.freeze_conv:
            self.train_ops["step_data_cache"] = [self.vision_ops.encoder_output]
            self.train_ops["step_data_cache_names"] = ["self.vision_ops.encoder_output"]
        else:
            self.train_ops["step_data_cache"] = []
            self.train_ops["step_data_cache_names"] = []

        # Set up blobs that are needed for the computation in rest of the graph.
        self.ego_map_ops = self.vision_ops.fss_logits
        self.coverage_ops = self.vision_ops.confs_probs

        # Zero pad these to make them same size as what the planner expects.
        for i in range(len(self.ego_map_ops)):
            if args.mapper_arch.pad_map_with_zeros_each[i] > 0:
                paddings = np.zeros((5, 2), dtype=np.int32)
                paddings[2:4, :] = args.mapper_arch.pad_map_with_zeros_each[i]
                paddings_op = tf.constant(paddings, dtype=tf.int32)
                self.ego_map_ops[i] = tf.pad(self.ego_map_ops[i], paddings=paddings_op)
                self.coverage_ops[i] = tf.pad(
                    self.coverage_ops[i], paddings=paddings_op
                )

        num_steps = task_params.num_steps
        if only_single_step_graph:
            num_steps = 1

        map_crop_size_ops = []
        for map_crop_size in task_params.map_crop_sizes:
            map_crop_size_ops.append(
                tf.constant(map_crop_size, dtype=tf.int32, shape=(2,))
            )

        with tf.name_scope("check_size"):
            is_single_step = tf.equal(
                tf.unstack(tf.shape(self.ego_map_ops[0]), num=5)[1], 1
            )

        fr_ops = []
        value_ops = []
        fr_intermediate_ops = []
        value_intermediate_ops = []
        crop_value_ops = []
        resize_crop_value_ops = []
        confs = []
        occupancys = []
        previous_value_op = None
        updated_state = []
        state_names = []

        for i in range(len(task_params.map_crop_sizes)):
            map_crop_size = task_params.map_crop_sizes[i]
            with tf.variable_scope("scale_{:d}".format(i)):
                # Accumulate the map.
                def fn(ns):
                    return running_combine(
                        self.ego_map_ops[i],
                        self.coverage_ops[i],
                        self.input_tensors["step"]["incremental_locs"]
                        * task_params.map_scales[i],
                        self.input_tensors["step"]["incremental_thetas"],
                        self.input_tensors["step"]["running_sum_num_{:d}".format(i)],
                        self.input_tensors["step"]["running_sum_denom_{:d}".format(i)],
                        self.input_tensors["step"]["running_max_denom_{:d}".format(i)],
                        map_crop_size,
                        ns,
                    )

                running_sum_num, running_sum_denom, running_max_denom = tf.cond(
                    is_single_step, lambda: fn(1), lambda: fn(num_steps)
                )
                updated_state += [running_sum_num, running_sum_denom, running_max_denom]
                state_names += [
                    "running_sum_num_{:d}".format(i),
                    "running_sum_denom_{:d}".format(i),
                    "running_max_denom_{:d}".format(i),
                ]

                # Concat the accumulated map and goal
                occupancy = running_sum_num / tf.maximum(running_sum_denom, 0.001)
                conf = running_max_denom

                # Concat occupancy, how much occupied and goal.
                with tf.name_scope("concat"):
                    sh = [-1, map_crop_size, map_crop_size, task_params.map_channels]
                    occupancy = tf.reshape(occupancy, shape=sh)
                    conf = tf.reshape(conf, shape=sh)

                    sh = [-1, map_crop_size, map_crop_size, task_params.goal_channels]
                    goal = tf.reshape(
                        self.input_tensors["step"]["ego_goal_imgs_{:d}".format(i)],
                        shape=sh,
                    )
                    to_concat = [occupancy, conf, goal]

                    if previous_value_op is not None:
                        to_concat.append(previous_value_op)

                    x = tf.concat(to_concat, 3)

                # Pass the map, previous rewards and the goal through a few convolutional
                # layers to get fR.
                fr_op, fr_intermediate_op = fr_v2(
                    x,
                    output_neurons=args.arch.fr_neurons,
                    inside_neurons=args.arch.fr_inside_neurons,
                    is_training=batch_norm_is_training_op,
                    name="fr",
                    wt_decay=args.solver.wt_decay,
                    stride=args.arch.fr_stride,
                )

                # Do Value Iteration on the fR
                value_op, value_intermediate_op = value_iteration_network(
                    fr_op,
                    num_iters=args.arch.vin_num_iters,
                    val_neurons=args.arch.vin_val_neurons,
                    action_neurons=args.arch.vin_action_neurons,
                    kernel_size=args.arch.vin_ks,
                    share_wts=args.arch.vin_share_wts,
                    name="vin",
                    wt_decay=args.solver.wt_decay,
                )

                # Crop out and upsample the previous value map.
                remove = args.arch.crop_remove_each
                if remove > 0:
                    crop_value_op = value_op[:, remove:-remove, remove:-remove, :]
                else:
                    crop_value_op = value_op
                crop_value_op = tf.reshape(
                    crop_value_op,
                    shape=[
                        -1,
                        args.arch.value_crop_size,
                        args.arch.value_crop_size,
                        args.arch.vin_val_neurons,
                    ],
                )
                if i < len(task_params.map_crop_sizes) - 1:
                    # Reshape it to shape of the next scale.
                    previous_value_op = tf.image.resize_bilinear(
                        crop_value_op, map_crop_size_ops[i + 1], align_corners=True
                    )
                    resize_crop_value_ops.append(previous_value_op)

                occupancys.append(occupancy)
                confs.append(conf)
                value_ops.append(value_op)
                crop_value_ops.append(crop_value_op)
                fr_ops.append(fr_op)
                fr_intermediate_ops.append(fr_intermediate_op)

        self.value_ops = value_ops
        self.value_intermediate_ops = value_intermediate_ops
        self.fr_ops = fr_ops
        self.fr_intermediate_ops = fr_intermediate_ops
        self.final_value_op = crop_value_op
        self.crop_value_ops = crop_value_ops
        self.resize_crop_value_ops = resize_crop_value_ops
        self.confs = confs
        self.occupancys = occupancys

        sh = [-1, args.arch.vin_val_neurons * ((args.arch.value_crop_size) ** 2)]
        self.value_features_op = tf.reshape(
            self.final_value_op, sh, name="reshape_value_op"
        )

        # Determine what action to take.
        with tf.variable_scope("action_pred"):
            batch_norm_param = args.arch.pred_batch_norm_param
            batch_norm_param["activation_fn"] = getattr(
                tf.nn, batch_norm_param["activation_fn"]
            )
            if batch_norm_param is not None:
                batch_norm_param["is_training"] = batch_norm_is_training_op
            self.action_logits_op, _ = fc_network(
                self.value_features_op,
                neurons=args.arch.pred_neurons,
                wt_decay=args.solver.wt_decay,
                name="pred",
                offset=0,
                num_pred=task_params.num_actions,
                batch_norm_param=batch_norm_param,
            )
            self.action_prob_op = tf.nn.softmax(self.action_logits_op)

        init_state = tf.constant(
            0.0,
            dtype=tf.float32,
            shape=[
                task_params.batch_size,
                1,
                map_crop_size,
                map_crop_size,
                task_params.map_channels,
            ],
        )

        self.train_ops["state_names"] = state_names
        self.train_ops["updated_state"] = updated_state
        self.train_ops["init_state"] = [init_state for __ in updated_state]

        self.train_ops["step"] = self.action_prob_op
        self.train_ops["batch_norm_is_training_op"] = batch_norm_is_training_op_ph

        self.init_op = tf.group(
            tf.global_variables_initializer(), tf.local_variables_initializer()
        )
        self.saver_op = tf.train.Saver(
            keep_checkpoint_every_n_hours=4, write_version=tf.train.SaverDef.V2
        )

    def setup_inputs(
        self,
        batch_size,
        img_height,
        img_width,
        img_channels,
        map_crop_sizes,
        map_channels,
        goal_channels,
    ):
        with tf.name_scope("inputs"):
            inputs = []
            inputs.append(
                (
                    "imgs",
                    tf.float32,
                    (batch_size, None, 1, img_height, img_width, img_channels),
                )
            )

            for i in range(len(map_crop_sizes)):
                inputs.append(
                    (
                        "ego_goal_imgs_{:d}".format(i),
                        tf.float32,
                        (
                            batch_size,
                            None,
                            map_crop_sizes[i],
                            map_crop_sizes[i],
                            goal_channels,
                        ),
                    )
                )
                for s in ["sum_num", "sum_denom", "max_denom"]:
                    inputs.append(
                        (
                            "running_" + s + "_{:d}".format(i),
                            tf.float32,
                            (
                                batch_size,
                                1,
                                map_crop_sizes[i],
                                map_crop_sizes[i],
                                map_channels,
                            ),
                        )
                    )

            inputs.append(("incremental_locs", tf.float32, (batch_size, None, 2)))
            inputs.append(("incremental_thetas", tf.float32, (batch_size, None, 1)))
            step_input_data, _ = setup_inputs(inputs)
        return step_input_data

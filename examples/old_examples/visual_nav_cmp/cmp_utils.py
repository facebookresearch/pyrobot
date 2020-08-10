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

"""Utility functions for setting up the CMP graph.
"""
import numpy as np
import tensorflow as tf
from recordclass import recordclass
from tensorflow.contrib import slim

from tf_utils import dense_resample, resnet_v2_50, fc_network
from tf_utils import resnet_v2, custom_residual_block, get_flow


def get_repr_from_image(
    images_reshaped,
    modalities,
    data_augment,
    encoder,
    freeze_conv,
    wt_decay,
    is_training,
):
    # Pass image through lots of convolutional layers, to obtain pool5
    with tf.name_scope("pre_rgb"):
        # Convert to brightness between 0 and 1.
        x = (images_reshaped + 128.0) / 255.0
        x = (x - 0.5) * 2.0
    scope_name = encoder
    resnet_is_training = is_training and (not freeze_conv)
    with slim.arg_scope(resnet_v2.resnet_utils.resnet_arg_scope(weight_decay=wt_decay)):
        assert encoder == "resnet_v2_50"
        x, end_points = resnet_v2_50(
            x,
            num_classes=None,
            global_pool=False,
            output_stride=None,
            reuse=None,
            scope=scope_name,
            is_training=resnet_is_training,
        )
    vars_ = slim.get_variables_to_restore()
    conv_feat = x
    return conv_feat, vars_


def value_iteration_network(
    fr,
    num_iters,
    val_neurons,
    action_neurons,
    kernel_size,
    share_wts=False,
    name="vin",
    wt_decay=0.0001,
    activation_fn=None,
    shape_aware=False,
):
    """
    Constructs a Value Iteration Network, convolutions and max pooling across
    channels.
    Input:
      fr:             NxWxHxC
      val_neurons:    Number of channels for maintaining the value.
      action_neurons: Computes action_neurons * val_neurons at each iteration to
                      max pool over.
    Output:
      value image:  NxHxWx(val_neurons)
    """
    init_var = np.sqrt(2.0 / (kernel_size ** 2) / (val_neurons * action_neurons))
    vals = []
    with tf.variable_scope(name) as varscope:
        if not shape_aware:
            fr_shape = tf.unstack(tf.shape(fr))
            val_shape = tf.stack(fr_shape[:-1] + [val_neurons])
            val = tf.zeros(val_shape, name="val_init")
        else:
            val = tf.expand_dims(tf.zeros_like(fr[:, :, :, 0]), dim=-1) * tf.constant(
                0.0, dtype=tf.float32, shape=[1, 1, 1, val_neurons]
            )
            val_shape = tf.shape(val)
        vals.append(val)
        for i in range(num_iters):
            if share_wts:
                # The first Value Iteration maybe special, so it can have its own
                # paramterss.
                scope = "conv"
                if i == 0:
                    scope = "conv_0"
                if i > 1:
                    varscope.reuse_variables()
            else:
                scope = "conv_{:d}".format(i)
            val = slim.conv2d(
                tf.concat([val, fr], 3, name="concat_{:d}".format(i)),
                num_outputs=action_neurons * val_neurons,
                kernel_size=kernel_size,
                stride=1,
                activation_fn=activation_fn,
                scope=scope,
                normalizer_fn=None,
                weights_regularizer=slim.l2_regularizer(wt_decay),
                weights_initializer=tf.random_normal_initializer(stddev=init_var),
                biases_initializer=tf.zeros_initializer(),
            )
            val = tf.reshape(
                val, [-1, action_neurons * val_neurons, 1, 1], name="re_{:d}".format(i)
            )
            val = slim.max_pool2d(
                val,
                kernel_size=[action_neurons, 1],
                stride=[action_neurons, 1],
                padding="VALID",
                scope="val_{:d}".format(i),
            )
            val = tf.reshape(val, val_shape, name="unre_{:d}".format(i))
            vals.append(val)
    return val, vals


def rotate_preds(loc_on_map, relative_theta, map_size, preds, output_valid_mask):
    with tf.name_scope("rotate"):
        flow_op = get_flow(loc_on_map, relative_theta, map_size=map_size)
        if type(preds) != list:
            rotated_preds, valid_mask_warps = dense_resample(
                preds, flow_op, output_valid_mask
            )
        else:
            rotated_preds = []
            valid_mask_warps = []
            for pred in preds:
                rotated_pred, valid_mask_warp = dense_resample(
                    pred, flow_op, output_valid_mask
                )
                rotated_preds.append(rotated_pred)
                valid_mask_warps.append(valid_mask_warp)
    return rotated_preds, valid_mask_warps


def deconv(
    x,
    is_training,
    wt_decay,
    neurons,
    strides,
    layers_per_block,
    kernel_size,
    conv_fn,
    name,
    offset=0,
):
    """Generates a up sampling network with residual connections.
    """
    batch_norm_param = {
        "center": True,
        "scale": True,
        "activation_fn": tf.nn.relu,
        "is_training": is_training,
    }
    outs = []
    for i, (neuron, stride) in enumerate(zip(neurons, strides)):
        for s in range(layers_per_block):
            scope = "{:s}_{:d}_{:d}".format(name, i + 1 + offset, s + 1)
            x = custom_residual_block(
                x,
                neuron,
                kernel_size,
                stride,
                scope,
                is_training,
                wt_decay,
                use_residual=True,
                residual_stride_conv=True,
                conv_fn=conv_fn,
                batch_norm_param=batch_norm_param,
            )
            stride = 1
        outs.append((x, True))
    return x, outs


def fr_v2(
    x,
    output_neurons,
    inside_neurons,
    is_training,
    name="fr",
    wt_decay=0.0001,
    stride=1,
    updates_collections=tf.GraphKeys.UPDATE_OPS,
):
    """Performs fusion of information between the map and the reward map.
    Inputs
      x:   NxHxWxC1

    Outputs
      fr map:     NxHxWx(output_neurons)
    """
    if type(stride) != list:
        stride = [stride]
    with slim.arg_scope(resnet_v2.resnet_utils.resnet_arg_scope(weight_decay=wt_decay)):
        with slim.arg_scope(
            [slim.batch_norm], updates_collections=updates_collections
        ) as arg_sc:
            # Change the updates_collections for the conv normalizer_params to None
            for i in range(len(arg_sc.keys())):
                if "convolution" in arg_sc.keys()[i]:
                    if "normalizer_params" in arg_sc.values()[i]:
                        arg_sc.values()[i]["normalizer_params"][
                            "updates_collections"
                        ] = updates_collections
                    else:
                        assert updates_collections == "update_ops"

            with slim.arg_scope(arg_sc):
                bottleneck = resnet_v2.bottleneck
                blocks = []
                for i, s in enumerate(stride):
                    b = resnet_v2.resnet_utils.Block(
                        "block{:d}".format(i + 1),
                        bottleneck,
                        [
                            {
                                "depth": output_neurons,
                                "depth_bottleneck": inside_neurons,
                                "stride": stride[i],
                            }
                        ],
                    )
                    blocks.append(b)
                x, outs = resnet_v2.resnet_v2(
                    x,
                    blocks,
                    num_classes=None,
                    global_pool=False,
                    output_stride=None,
                    include_root_block=False,
                    reuse=False,
                    scope=name,
                    is_training=is_training,
                )
    return x, outs


def running_combine(
    fss_logits,
    confs_probs,
    incremental_locs,
    incremental_thetas,
    previous_sum_num,
    previous_sum_denom,
    previous_max_denom,
    map_size,
    num_steps,
):
    # fss_logits is B x N x H x W x C
    # confs_logits is B x N x H x W x C
    # incremental_locs is B x N x 2
    # incremental_thetas is B x N x 1
    # previous_sum_num etc is B x 1 x H x W x C

    with tf.name_scope("combine_{:d}".format(num_steps)):
        running_sum_nums_ = []
        running_sum_denoms_ = []
        running_max_denoms_ = []

        fss_logits_ = tf.unstack(fss_logits, axis=1, num=num_steps)
        confs_probs_ = tf.unstack(confs_probs, axis=1, num=num_steps)
        incremental_locs_ = tf.unstack(incremental_locs, axis=1, num=num_steps)
        incremental_thetas_ = tf.unstack(incremental_thetas, axis=1, num=num_steps)
        running_sum_num = tf.unstack(previous_sum_num, axis=1, num=1)[0]
        running_sum_denom = tf.unstack(previous_sum_denom, axis=1, num=1)[0]
        running_max_denom = tf.unstack(previous_max_denom, axis=1, num=1)[0]

        for i in range(num_steps):
            # Rotate the previous running_num and running_denom
            running_sum_num, running_sum_denom, running_max_denom = rotate_preds(
                incremental_locs_[i],
                incremental_thetas_[i],
                map_size,
                [running_sum_num, running_sum_denom, running_max_denom],
                output_valid_mask=False,
            )[0]
            # print i, num_steps, running_sum_num.get_shape().as_list()
            running_sum_num = running_sum_num + fss_logits_[i] * confs_probs_[i]
            running_sum_denom = running_sum_denom + confs_probs_[i]
            running_max_denom = tf.maximum(running_max_denom, confs_probs_[i])
            running_sum_nums_.append(running_sum_num)
            running_sum_denoms_.append(running_sum_denom)
            running_max_denoms_.append(running_max_denom)

        running_sum_nums = tf.stack(running_sum_nums_, axis=1)
        running_sum_denoms = tf.stack(running_sum_denoms_, axis=1)
        running_max_denoms = tf.stack(running_max_denoms_, axis=1)
        return running_sum_nums, running_sum_denoms, running_max_denoms


def get_map_from_images(
    imgs,
    mapper_arch,
    task_params,
    freeze_conv,
    wt_decay,
    is_training,
    batch_norm_is_training_op,
    num_maps,
    split_maps=True,
):
    # Hit image with a resnet.
    n_views = 1
    MapperTensors = recordclass(
        "MapperTensors",
        "vars_to_restore encoder_output conv_feat reshape_conv_feat deconv_output fss_logits confs_logits confs_probs",
    )
    out = MapperTensors(*[None for _ in range(8)])

    images_reshaped = tf.reshape(
        imgs,
        shape=[
            -1,
            task_params.img_height,
            task_params.img_width,
            task_params.img_channels,
        ],
        name="re_image",
    )

    x, out.vars_to_restore = get_repr_from_image(
        images_reshaped,
        ["rgb"],
        task_params.data_augment,
        mapper_arch.encoder,
        freeze_conv,
        wt_decay,
        is_training,
    )

    # Reshape into nice things so that these can be accumulated over time steps
    # for faster backprop.
    sh_before = x.get_shape().as_list()
    out.encoder_output = tf.reshape(
        x, shape=[task_params.batch_size, -1, n_views] + sh_before[1:]
    )
    x = tf.reshape(out.encoder_output, shape=[-1] + sh_before[1:])

    # Add a layer to reduce dimensions for a fc layer.
    if mapper_arch.dim_reduce_neurons > 0:
        ks = 1
        neurons = mapper_arch.dim_reduce_neurons
        init_var = np.sqrt(2.0 / (ks ** 2) / neurons)
        batch_norm_param = mapper_arch.batch_norm_param
        batch_norm_param["activation_fn"] = getattr(
            tf.nn, batch_norm_param["activation_fn"]
        )
        batch_norm_param["is_training"] = batch_norm_is_training_op
        out.conv_feat = slim.conv2d(
            x,
            neurons,
            kernel_size=ks,
            stride=1,
            normalizer_fn=slim.batch_norm,
            normalizer_params=batch_norm_param,
            padding="SAME",
            scope="dim_reduce",
            weights_regularizer=slim.l2_regularizer(wt_decay),
            weights_initializer=tf.random_normal_initializer(stddev=init_var),
        )
        reshape_conv_feat = slim.flatten(out.conv_feat)
        sh = reshape_conv_feat.get_shape().as_list()
        out.reshape_conv_feat = tf.reshape(
            reshape_conv_feat, shape=[-1, sh[1] * n_views]
        )

    with tf.variable_scope("fc"):
        # Fully connected layers to compute the representation in top-view space.
        fc_batch_norm_param = {
            "center": True,
            "scale": True,
            "activation_fn": tf.nn.relu,
            "is_training": batch_norm_is_training_op,
        }
        f = out.reshape_conv_feat
        out_neurons = (mapper_arch.fc_out_size ** 2) * mapper_arch.fc_out_neurons
        neurons = mapper_arch.fc_neurons + [out_neurons]
        f, _ = fc_network(
            f,
            neurons=neurons,
            wt_decay=wt_decay,
            name="fc",
            offset=0,
            batch_norm_param=fc_batch_norm_param,
            is_training=is_training,
            dropout_ratio=mapper_arch.fc_dropout,
        )
        f = tf.reshape(
            f,
            shape=[
                -1,
                mapper_arch.fc_out_size,
                mapper_arch.fc_out_size,
                mapper_arch.fc_out_neurons,
            ],
            name="re_fc",
        )

    # Use pool5 to predict the free space map via deconv layers.
    with tf.variable_scope("deconv"):
        x, outs = deconv(
            f,
            batch_norm_is_training_op,
            wt_decay=wt_decay,
            neurons=mapper_arch.deconv_neurons,
            strides=mapper_arch.deconv_strides,
            layers_per_block=mapper_arch.deconv_layers_per_block,
            kernel_size=mapper_arch.deconv_kernel_size,
            conv_fn=slim.conv2d_transpose,
            offset=0,
            name="deconv",
        )

    # Reshape x the right way.
    sh = x.get_shape().as_list()
    x = tf.reshape(x, shape=[task_params.batch_size, -1] + sh[1:])
    out.deconv_output = x

    # Separate out the map and the confidence predictions, pass the confidence
    # through a sigmoid.
    if split_maps:
        with tf.name_scope("split"):
            out_all = tf.split(value=x, axis=4, num_or_size_splits=2 * num_maps)
            out.fss_logits = out_all[:num_maps]
            out.confs_logits = out_all[num_maps:]
        with tf.name_scope("sigmoid"):
            out.confs_probs = [tf.nn.sigmoid(__) for __ in out.confs_logits]
    return out

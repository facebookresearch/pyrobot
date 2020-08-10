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

import numpy as np
import tensorflow as tf
from tensorflow.contrib import slim
from tensorflow.contrib.slim import arg_scope
from tensorflow.contrib.slim.nets import resnet_v2

resnet_v2_50 = resnet_v2.resnet_v2_50


def custom_residual_block(
    x,
    neurons,
    kernel_size,
    stride,
    name,
    is_training,
    wt_decay=0.0001,
    use_residual=True,
    residual_stride_conv=True,
    conv_fn=slim.conv2d,
    batch_norm_param=None,
):
    # batch norm x and relu
    init_var = np.sqrt(2.0 / (kernel_size ** 2) / neurons)
    with arg_scope(
        [conv_fn],
        weights_regularizer=slim.l2_regularizer(wt_decay),
        weights_initializer=tf.random_normal_initializer(stddev=init_var),
        biases_initializer=tf.zeros_initializer(),
    ):

        if batch_norm_param is None:
            batch_norm_param = {
                "center": True,
                "scale": False,
                "activation_fn": tf.nn.relu,
                "is_training": is_training,
            }

        y = slim.batch_norm(x, scope=name + "_bn", **batch_norm_param)

        y = conv_fn(
            y,
            num_outputs=neurons,
            kernel_size=kernel_size,
            stride=stride,
            activation_fn=None,
            scope=name + "_1",
            normalizer_fn=slim.batch_norm,
            normalizer_params=batch_norm_param,
        )

        y = conv_fn(
            y,
            num_outputs=neurons,
            kernel_size=kernel_size,
            stride=1,
            activation_fn=None,
            scope=name + "_2",
        )

        if use_residual:
            if stride != 1 or x.get_shape().as_list()[-1] != neurons:
                batch_norm_param_ = dict(batch_norm_param)
                batch_norm_param_["activation_fn"] = None
                x = conv_fn(
                    x,
                    num_outputs=neurons,
                    kernel_size=1,
                    stride=stride if residual_stride_conv else 1,
                    activation_fn=None,
                    scope=name + "_0_1x1",
                    normalizer_fn=slim.batch_norm,
                    normalizer_params=batch_norm_param_,
                )
                if not residual_stride_conv:
                    x = slim.avg_pool2d(x, 1, stride=stride, scope=name + "_0_avg")

            y = tf.add(x, y, name=name + "_add")

        return y


def dense_resample(im, flow_im, output_valid_mask, name="dense_resample"):
    """ Resample reward at particular locations.
    Args:
      im:      ...xHxWxC matrix to sample from.
      flow_im: ...xHxWx2 matrix, samples the image using absolute offsets as given
               by the flow_im.
    """
    with tf.name_scope(name):
        valid_mask = None

        x, y = tf.unstack(flow_im, axis=-1)
        x = tf.cast(tf.reshape(x, [-1]), tf.float32)
        y = tf.cast(tf.reshape(y, [-1]), tf.float32)

        # constants
        shape = tf.unstack(tf.shape(im))
        channels = shape[-1]
        width = shape[-2]
        height = shape[-3]
        num_batch = tf.cast(tf.reduce_prod(tf.stack(shape[:-3])), "int32")
        zero = tf.constant(0, dtype=tf.int32)

        # Round up and down.
        x0 = tf.cast(tf.floor(x), "int32")
        x1 = x0 + 1
        y0 = tf.cast(tf.floor(y), "int32")
        y1 = y0 + 1

        if output_valid_mask:
            valid_mask = tf.logical_and(
                tf.logical_and(
                    tf.less_equal(x, tf.cast(width, tf.float32) - 1.0),
                    tf.greater_equal(x, 0.0),
                ),
                tf.logical_and(
                    tf.less_equal(y, tf.cast(height, tf.float32) - 1.0),
                    tf.greater_equal(y, 0.0),
                ),
            )
            valid_mask = tf.reshape(valid_mask, shape=shape[:-1] + [1])

        x0 = tf.clip_by_value(x0, zero, width - 1)
        x1 = tf.clip_by_value(x1, zero, width - 1)
        y0 = tf.clip_by_value(y0, zero, height - 1)
        y1 = tf.clip_by_value(y1, zero, height - 1)

        dim2 = width
        dim1 = width * height

        # Create base index
        base = tf.reshape(tf.range(num_batch) * dim1, shape=[-1, 1])
        base = base + tf.expand_dims(tf.zeros([height * width], dtype=tf.int32), 0)
        base = tf.reshape(base, shape=[-1])
        # base = tf.reshape(tf.tile(base, [tf.constant(1), height*width], name='tttile'), shape=[-1])

        base_y0 = base + y0 * dim2
        base_y1 = base + y1 * dim2
        idx_a = base_y0 + x0
        idx_b = base_y1 + x0
        idx_c = base_y0 + x1
        idx_d = base_y1 + x1

        # use indices to lookup pixels in the flat image and restore channels dim
        sh = tf.stack([tf.constant(-1, dtype=tf.int32), channels])
        im_flat = tf.cast(tf.reshape(im, sh), dtype=tf.float32)
        pixel_a = tf.gather(im_flat, idx_a)
        pixel_b = tf.gather(im_flat, idx_b)
        pixel_c = tf.gather(im_flat, idx_c)
        pixel_d = tf.gather(im_flat, idx_d)

        # and finally calculate interpolated values
        x1_f = tf.to_float(x1)
        y1_f = tf.to_float(y1)

        wa = tf.expand_dims(((x1_f - x) * (y1_f - y)), 1)
        wb = tf.expand_dims((x1_f - x) * (1.0 - (y1_f - y)), 1)
        wc = tf.expand_dims(((1.0 - (x1_f - x)) * (y1_f - y)), 1)
        wd = tf.expand_dims(((1.0 - (x1_f - x)) * (1.0 - (y1_f - y))), 1)

        output = tf.add_n([wa * pixel_a, wb * pixel_b, wc * pixel_c, wd * pixel_d])
        output = tf.reshape(output, shape=tf.shape(im))
        return output, valid_mask


def get_flow(t, theta, map_size, name_scope="gen_flow"):
    """
    Rotates the map by theta and translates the rotated map by t.

    Assume that the robot rotates by an angle theta and then moves forward by
    translation t. This function returns the flow field field. For every pixel in
    the new image it tells us which pixel in the original image it came from:
    NewI(x, y) = OldI(flow_x(x,y), flow_y(x,y)).

    Assume there is a point p in the original image. Robot rotates by R and moves
    forward by t.  p1 = Rt*p; p2 = p1 - t; (the world moves in opposite direction.
    So, p2 = Rt*p - t, thus p2 came from R*(p2+t), which is what this function
    calculates.

      t:      ... x 2 (translation for B batches of N motions each).
      theta:  ... x 1 (rotation for B batches of N motions each).

      Output: ... x map_size x map_size x 2
    """

    with tf.name_scope(name_scope):
        tx, ty = tf.unstack(tf.reshape(t, shape=[-1, 1, 1, 1, 2]), axis=4)
        theta = tf.reshape(theta, shape=[-1, 1, 1, 1])
        c = tf.constant((map_size - 1.0) / 2.0, dtype=tf.float32)

        x, y = np.meshgrid(np.arange(map_size), np.arange(map_size))
        x = tf.constant(
            x[np.newaxis, :, :, np.newaxis],
            dtype=tf.float32,
            name="x",
            shape=[1, map_size, map_size, 1],
        )
        y = tf.constant(
            y[np.newaxis, :, :, np.newaxis],
            dtype=tf.float32,
            name="y",
            shape=[1, map_size, map_size, 1],
        )

        x = x - (-tx + c)
        y = y - (-ty + c)

        sin_theta = tf.sin(theta)
        cos_theta = tf.cos(theta)
        xr = cos_theta * x - sin_theta * y
        yr = sin_theta * x + cos_theta * y

        xr = xr + c
        yr = yr + c

        flow = tf.stack([xr, yr], axis=-1)
        sh = tf.unstack(tf.shape(t), axis=0)
        sh = tf.stack(
            sh[:-1] + [tf.constant(_, dtype=tf.int32) for _ in [map_size, map_size, 2]]
        )
        flow = tf.reshape(flow, shape=sh)
        return flow


def fc_network(
    x,
    neurons,
    wt_decay,
    name,
    num_pred=None,
    offset=0,
    batch_norm_param=None,
    dropout_ratio=0.0,
    is_training=None,
):
    if dropout_ratio > 0:
        assert (
            is_training is not None
        ), "is_training needs to be defined when trainnig with dropout."

    repr = []
    for i, neuron in enumerate(neurons):
        init_var = np.sqrt(2.0 / neuron)
        if batch_norm_param is not None:
            x = slim.fully_connected(
                x,
                neuron,
                activation_fn=None,
                weights_initializer=tf.random_normal_initializer(stddev=init_var),
                weights_regularizer=slim.l2_regularizer(wt_decay),
                normalizer_fn=slim.batch_norm,
                normalizer_params=batch_norm_param,
                biases_initializer=tf.zeros_initializer(),
                scope="{:s}_{:d}".format(name, offset + i),
            )
        else:
            x = slim.fully_connected(
                x,
                neuron,
                activation_fn=tf.nn.relu,
                weights_initializer=tf.random_normal_initializer(stddev=init_var),
                weights_regularizer=slim.l2_regularizer(wt_decay),
                biases_initializer=tf.zeros_initializer(),
                scope="{:s}_{:d}".format(name, offset + i),
            )
        if dropout_ratio > 0:
            x = slim.dropout(
                x,
                keep_prob=1 - dropout_ratio,
                is_training=is_training,
                scope="{:s}_{:d}".format("dropout_" + name, offset + i),
            )
        repr.append(x)

    if num_pred is not None:
        init_var = np.sqrt(2.0 / num_pred)
        x = slim.fully_connected(
            x,
            num_pred,
            weights_regularizer=slim.l2_regularizer(wt_decay),
            weights_initializer=tf.random_normal_initializer(stddev=init_var),
            biases_initializer=tf.zeros_initializer(),
            activation_fn=None,
            scope="{:s}_pred".format(name),
        )
    return x, repr


def setup_inputs(inputs):
    input_tensors = {}
    input_shapes = {}
    for (name, typ, sz) in inputs:
        _ = tf.placeholder(typ, shape=sz, name=name)
        input_tensors[name] = _
        input_shapes[name] = sz
    return input_tensors, input_shapes


def prepare_feed_dict(input_tensors, inputs):
    feed_dict = {}
    for n in input_tensors.keys():
        feed_dict[input_tensors[n]] = inputs[n].astype(
            input_tensors[n].dtype.as_numpy_dtype
        )
    return feed_dict

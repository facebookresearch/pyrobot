# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# my_project/config.py
import numpy as np
from yacs.config import CfgNode as CN

_C = CN()

data_augment = CN()
data_augment.structured = False
data_augment.delta_z = 0.0
data_augment.relight = False
data_augment.lr_flip = 0
data_augment.delta_xy = 0.0
data_augment.delta_elevation = 0.0
data_augment.relight_fast = False
data_augment.delta_angle = 0.0

task_params = CN()
task_params.img_height = 240
task_params.img_width = 320
task_params.img_channels = 3
task_params.num_actions = 4
task_params.batch_size = 1
task_params.map_crop_sizes = [16 for _ in range(3)]
task_params.map_scales = [np.power(2.0, i - 5) for i in range(3)]
task_params.map_channels = 8
task_params.num_steps = 40
task_params.goal_channels = 4
task_params.data_augment = data_augment

navtask = CN()
navtask.task_params = task_params
_C.navtask = navtask

solver = CN()
solver.seed = 0
solver.wt_decay = 0.001
solver.freeze_conv = True
solver.pretrained_path = ""
_C.solver = solver

batch_norm_param = CN()
batch_norm_param.activation_fn = "relu"
batch_norm_param.scale = True
batch_norm_param.center = True

mapper_arch = CN()
mapper_arch.deconv_kernel_size = 4
mapper_arch.pad_map_with_zeros_each = [0, 0, 0]
mapper_arch.dim_reduce_neurons = 64
mapper_arch.fc_neurons = [1024, 1024]
mapper_arch.fc_out_neurons = 64
mapper_arch.deconv_strides = [1, 2, 1]
mapper_arch.encoder = "resnet_v2_50"
mapper_arch.deconv_neurons = [192, 96, 48]
mapper_arch.fc_dropout = 0.5
mapper_arch.combine_type = "wt_avg_logits"
mapper_arch.fc_out_size = 8
mapper_arch.deconv_layers_per_block = 2
mapper_arch.batch_norm_param = batch_norm_param

_C.mapper_arch = mapper_arch

pred_batch_norm_param = CN()
pred_batch_norm_param.activation_fn = "relu"
pred_batch_norm_param.scale = True
pred_batch_norm_param.center = True

arch = CN()
arch.pred_neurons = [64, 64]
arch.vin_ks = 3
arch.pred_batch_norm_param = pred_batch_norm_param
arch.vin_val_neurons = 3
arch.fr_neurons = 16
arch.fr_inside_neurons = 32
arch.fr_ver = "v2"
arch.vin_share_wts = False
arch.value_crop_size = 8
arch.crop_remove_each = 4
arch.multi_scale = True
arch.vin_num_iters = 8
arch.vin_action_neurons = 8
arch.fr_stride = 1
_C.arch = arch


def get_cfg_defaults():
    """Get a yacs CfgNode object with default values for my_project."""
    # Return a clone so that the defaults will not be altered
    # This is for the "local variable" use pattern
    return _C.clone()


# Alternatively, provide a way to import the defaults as
# a global singleton:
# cfg = _C  # users can `from config import cfg`

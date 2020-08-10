# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import torch
import torch.utils.model_zoo as model_zoo
from torch import nn
from torchvision.models.resnet import ResNet, BasicBlock, model_urls

n_class = 18
n_images = 9
n_robots = 5


class IfullRobHWNet(ResNet):
    def __init__(
        self,
        h_size=128,
        out_size=n_class,
        noise_size=n_images,
        pretrained_resnet18=True,
        fixed_feature=False,
        **kwargs
    ):
        super(IfullRobHWNet, self).__init__(BasicBlock, [2, 2, 2, 2])
        if pretrained_resnet18:
            self.load_from_pretrained_resnet18()
        if fixed_feature:
            print("FIXING RESNET PARAMETERS")
            for param in self.parameters():
                param.requires_grad = False
        num_ftrs = self.fc.in_features
        self.num_targets = 5
        self.num_targets_hw = 10
        self.fc_noise_0 = nn.Linear(num_ftrs, h_size)
        self.fc_noise_1 = nn.Linear(
            h_size + self.num_targets * n_robots + self.num_targets_hw * 2, h_size
        )
        self.fc_noise_2 = nn.Linear(h_size, h_size)
        self.fc_noise_3 = nn.Linear(h_size, noise_size)
        self.fc_noise_softmax = nn.Softmax(dim=None)
        self.fc_angle_0 = nn.Linear(num_ftrs, h_size)
        self.fc_angle_1 = nn.Linear(h_size, out_size)

    def forward(self, x, robot_one_hot_labels, h, w, full_x, **kwargs):
        one_hot_labels = robot_one_hot_labels
        full_pool_out = self._one_forward(full_x)
        pool_out = self._one_forward(x)
        fc_noise_0_out = self.relu(self.fc_noise_0(full_pool_out))
        label_info = one_hot_labels.float()
        label_info_mult = torch.cat([label_info] * self.num_targets, 1)
        h_info_mult = torch.stack([h] * self.num_targets_hw).transpose(1, 0)
        w_info_mult = torch.stack([w] * self.num_targets_hw).transpose(1, 0)
        fc_noise_in = torch.cat(
            [fc_noise_0_out, label_info_mult, h_info_mult, w_info_mult], 1
        )
        noise_x = self.fc_noise_1(fc_noise_in)
        noise_x = self.relu(noise_x)
        noise_x = self.fc_noise_2(noise_x)
        noise_x = self.relu(noise_x)
        noise_out = self.fc_noise_3(noise_x)
        fc_noise_soft = self.fc_noise_softmax(noise_out)
        fc_angle_0_out = self.fc_angle_0(pool_out)
        fc_angle_0_out = self.relu(fc_angle_0_out)
        fc_angle_1_out = self.fc_angle_1(fc_angle_0_out)
        return fc_angle_1_out, fc_noise_soft

    def _one_forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        x = self.avgpool(x)
        x = x.view(x.size(0), -1)
        return x

    def load_from_pretrained_resnet18(self):
        print("Loading pretrained resnet18")
        self.load_state_dict(model_zoo.load_url(model_urls["resnet18"]))

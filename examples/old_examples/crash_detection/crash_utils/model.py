# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import torch
import torch.utils.model_zoo as model_zoo
from torch import nn
from torchvision.models.resnet import ResNet, BasicBlock, model_urls


class CrashDetectorNet(ResNet):
    """
    """

    def __init__(self, pretrained_resnet18=True, H=32, **kwargs):
        super(CrashDetectorNet, self).__init__(BasicBlock, [2, 2, 2, 2])
        if pretrained_resnet18 == True:
            self.load_from_pretrained_resnet18()

        num_ftrs = self.fc.in_features
        self.fc_0 = torch.nn.Linear(num_ftrs, H)
        self.fc_1 = torch.nn.Linear(H, 2)
        self.fc_softmax = nn.Softmax(dim=None)

    def forward(self, x, **kwargs):
        pool_out = self._one_forward(x)
        fc_0_out = self.fc_0(pool_out)
        fc_0_out = self.relu(fc_0_out)
        fc_1_out = self.fc_1(fc_0_out)
        fc_soft = self.fc_softmax(fc_1_out)

        return fc_soft

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

#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Class definition for GraspTorchObj.
"""

import sys

import numpy as np
import torch
from PIL import Image
from torch.autograd import Variable
from torchvision import transforms

import deeper_models

sys.modules["patch_learner.deeper_models"] = deeper_models

is_gpu = torch.cuda.is_available()
n_class = 18
n_rob = 5


class GraspTorchObj(object):
    """
    This class contains functionality to wrap a torch grasp model with testing operations.
    """

    def __init__(self, model_path, transform=None):
        """
            The constructor for :class:`GraspTorchObj` class.
    
            :param model_path: Path where the grasp model should be loaded from
            :param transform: A PyTorch transform that gets applied on the input
            :type model_path: string
            :type transform: A torchvision.Transform object
            """
        torch.nn.Module.dump_patches = True
        self.model = torch.load(model_path).eval()
        self.image_size = 224
        if is_gpu:
            self.model = self.model.cuda()
        if transform is None:
            self.transform = self.get_default_transform()
        else:
            self.transform = transform

    def test_one_batch(self, x, h, w, angle_labels, robot_labels, full_x):
        """
        Runs the torch model on a batch of inputs.
    
            :param x: List of image patches
            :param h: List of heights of the center of the patch in the original image
            :param w: List of widths of the center of the patch in the original image
            :param angle_labels: List of labels corresponding to the angle to grasp
            :param robot_labels: List of robot_id corresponding to the robot
            :param full_x: List of original image where the patch comes from.
            :type x: list
            :type h: list
            :type w: list
            :type angle_labels: list
            :type robot_labels: list
            :type full_x: list
    
            :returns: List of grasp predictions
            :rtype: list
        """
        torch_patches_var = Variable(self.convert_cv2_patches(x))
        h_var = Variable(self.convert_hw(h))
        w_var = Variable(self.convert_hw(w))
        one_hot_var = self.convert_one_hot(angle_labels)
        robot_one_hot_var = self.convert_robot_one_hot(robot_labels)
        torch_images_var = Variable(self.convert_cv2_patches(full_x))

        if is_gpu:
            torch_patches_var = torch_patches_var.cuda()
            h_var = h_var.cuda()
            w_var = w_var.cuda()
            one_hot_var = one_hot_var.cuda()
            robot_one_hot_var = robot_one_hot_var.cuda()
            torch_images_var = torch_images_var.cuda()
        predictions, _ = self.model(
            x=torch_patches_var,
            h=h_var,
            w=w_var,
            one_hot_labels=one_hot_var,
            robot_one_hot_labels=robot_one_hot_var,
            full_x=torch_images_var,
        )
        return predictions.data.cpu().numpy()

    def convert_cv2_patches(self, P):
        """
        Converts a list of image patches to a torch Tensor
    
        :param P: List of image patches
        :type P: list
    
        :returns: A torch Tensor of image patches
        :rtype: torch Tensor
        """
        assert len(P.shape) == 4
        im_list = []
        for p in P:
            p = np.uint8(p)
            pil_im = Image.fromarray(p)
            pil_im = pil_im.convert("RGB")
            if self.transform:
                pil_im = self.transform(pil_im)
            im_list.append(pil_im)
        im_stacked = torch.stack(im_list)
        return im_stacked

    def convert_hw(self, h):
        """
        Converts a list of floats to a torch Tensor
    
        :param h: List of floats
        :type h: list
    
        :returns: A torch FloatTensor
        :rtype: torch FloatTensor
        """
        return torch.FloatTensor(h)

    def convert_one_hot(self, labels):
        """
        Converts a list of grasp angle labels to a Torch tensor of one_hot labels
    
        :param labels: List of angle labels
        :type labels: list
    
        :returns: A torch tensor of one_hot labels
        :rtype: torch Tensor
        """
        batch_size = len(labels)
        labels_tensor = Variable(torch.LongTensor(labels))
        y_onehot = Variable(torch.DoubleTensor(batch_size, n_class))
        y_onehot.zero_()
        labels_onehot = y_onehot.scatter_(1, labels_tensor.view(-1, 1), 1)
        return labels_onehot

    def convert_robot_one_hot(self, labels):
        """
        Converts a list of robot_id labels to a Torch tensor of one_hot labels
    
        :param labels: List of robot_id labels
        :type labels: list
    
        :returns: A torch tensor of one_hot labels
        :rtype: torch Tensor
        """
        batch_size = len(labels)
        labels_tensor = Variable(torch.LongTensor(labels))
        y_onehot = Variable(torch.DoubleTensor(batch_size, n_rob))
        y_onehot.zero_()
        labels_onehot = y_onehot.scatter_(1, labels_tensor.view(-1, 1), 1)
        return labels_onehot

    def get_default_transform(self):
        """
        Return the default transform to be applied on the images. This corresponds to the transform that was applied to the images during training.
    
        :returns: An image transform
        :rtype: a torchvision.Transform object
        """
        image_transforms = transforms.Compose(
            [
                transforms.Resize((self.image_size, self.image_size)),
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        return image_transforms

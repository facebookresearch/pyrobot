#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
"""
Class definition for Predictors
"""
import copy

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import numpy as np

n_class = 18
min_patch_std = 1
angle_dependence = [0.25, 0.5, 0.25]

try:
    xrange
except NameError:
    xrange = range


# Given image, returns image point and theta to grasp
class Predictors:
    """
    This class contains functionality to sample grasp patches from the scene image.
    """

    def __init__(self, img, grasp_obj=None):
        """
        The constructor for :class:`Predictors` class.
    
            :param img: An image of the scene
            :param grasp_obj: An object that contains grasp model operations
            :type img: np.ndarray
            :type grasp_obj: GraspTorchObj
        """
        self.img = img
        self.img_h, self.img_w, self.img_c = self.img.shape
        self.grasp_obj = grasp_obj

    def random_grasp(self):
        """
        Samples a random grasp configuration.
    
        :returns: A grasp configuration (height, width, angle) of a random grasp
        :rtype: tuple
        """
        h_g = np.random.randint(self.img_h)
        w_g = np.random.randint(self.img_w)
        t_g = np.random.randint(n_class)
        return h_g, w_g, t_g

    def center_grasp(self):
        """
        Select the center grasp configuration.
    
        :returns: A grasp configuration (height, width, angle) of the center grasp
        :rtype: tuple
        """
        h_g = np.int(self.img_h / 2)
        w_g = np.int(self.img_w / 2)
        t_g = np.int(n_class / 2)
        return h_g, w_g, t_g

    def graspNet_grasp(self, patch_size=300, num_samples=128):
        """
        Select grasp based on the grasp model.
    
        :param patch_size: Size of patch to be sampled
        :param num_samples: Number of patches to run
        :type patch_size: int
        :type num_samples: int
        """
        self.patch_size = patch_size
        half_patch_size = np.int(patch_size / 2) + 1
        h_range = self.img_h - patch_size - 2
        w_range = self.img_w - patch_size - 2

        # Initialize random patch points
        patch_hs = np.random.randint(h_range, size=num_samples) + half_patch_size
        patch_ws = np.random.randint(w_range, size=num_samples) + half_patch_size

        patch_Is = np.zeros((num_samples, patch_size, patch_size, self.img_c))
        patch_Is_resized = np.zeros(
            (
                num_samples,
                self.grasp_obj.image_size,
                self.grasp_obj.image_size,
                self.img_c,
            )
        )
        for looper in xrange(num_samples):
            isWhiteFlag = 1
            while isWhiteFlag == 1:
                patch_hs[looper] = np.random.randint(h_range) + half_patch_size
                patch_ws[looper] = np.random.randint(w_range) + half_patch_size
                h_b = patch_hs[looper] - half_patch_size
                w_b = patch_ws[looper] - half_patch_size
                patch_Is[looper] = self.img[
                    h_b : h_b + patch_size, w_b : w_b + patch_size
                ]
                # Make sure that the input has a minimal
                # amount of standard deviation from mean. If
                # not resample
                if patch_Is[looper].std() > min_patch_std:
                    isWhiteFlag = 0
                else:
                    isWhiteFlag = 1
            patch_Is_resized[looper] = cv2.resize(
                patch_Is[looper],
                (self.grasp_obj.image_size, self.grasp_obj.image_size),
                interpolation=cv2.INTER_CUBIC,
            )
        angle_labels = [0 for _ in patch_Is_resized]
        robot_labels = [0 for _ in patch_Is_resized]
        full_x = np.array([self.img for _ in patch_Is_resized])
        param_dict = {
            "x": patch_Is_resized,
            "h": patch_hs,
            "w": patch_ws,
            "angle_labels": angle_labels,
            "robot_labels": robot_labels,
            "full_x": full_x,
        }
        self.vals = self.grasp_obj.test_one_batch(**param_dict)

        # Normalizing angle uncertainity
        wf = angle_dependence
        self.norm_vals = copy.deepcopy(self.vals)
        for looper in xrange(num_samples):
            for norm_looper in xrange(n_class):
                n_val = (
                    wf[1] * self.norm_vals[looper, norm_looper]
                    + wf[0] * self.norm_vals[looper, (norm_looper - 1) % n_class]
                    + wf[2] * self.norm_vals[looper, (norm_looper + 1) % n_class]
                )
                self.norm_vals[looper, norm_looper] = n_val
        self.patch_hs = patch_hs
        self.patch_ws = patch_ws
        self.patch_Is_resized = patch_Is_resized

# /usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Class definition for GraspModel
"""
import errno
import os
import os.path
import time
from copy import deepcopy as copy

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import numpy as np
import rospy
from grasp_samplers.grasp_object import GraspTorchObj
from grasp_samplers.grasp_predictor import Predictors

dir_path = os.path.dirname(os.path.realpath(__file__))
SAVE_DIR = os.path.join(dir_path, "models")
GPU_ID = -1
MAX_BATCHSIZE = 20


def download_if_not_present(model_path, url):
    """
    Function that downloads a file from a url to a given location.

    :param model_path: Path where the file should be downlaoded to
    :param url: URL from where the file will be downloaded from
    :type model_path: string
    :type url: string
    """
    if not os.path.isfile(model_path):
        if not os.path.exists(os.path.dirname(model_path)):
            try:
                os.makedirs(os.path.dirname(model_path))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        print("GRASP MODEL NOT FOUND! DOWNLOADING IT!")
        os.system("wget {} -O {}".format(url, model_path))


def drawRectangle(I, h, w, t, gsize=300):
    """
    Function to draw a grasp rectangle on an image.

    :param I: Image on which the grasp will be drawn on
    :param h: height index for the center of grasp
    :param w: width index for the center of grasp
    :param t: grasp angle number of the grasp
    :param gsize: size of the grasp
    :type I: np.ndarray
    :type h: int
    :type w: int
    :type t: int
    :type gsize: int

    :returns: Image with grasp rectangle drawn on it
    :rtype: np.ndarray
    """
    I_temp = I
    grasp_l = gsize / 2.5
    grasp_w = gsize / 5.0
    grasp_angle = t * (np.pi / 18) - np.pi / 2

    points = np.array(
        [
            [-grasp_l, -grasp_w],
            [grasp_l, -grasp_w],
            [grasp_l, grasp_w],
            [-grasp_l, grasp_w],
        ]
    )
    R = np.array(
        [
            [np.cos(grasp_angle), -np.sin(grasp_angle)],
            [np.sin(grasp_angle), np.cos(grasp_angle)],
        ]
    )
    rot_points = np.dot(R, points.transpose()).transpose()
    im_points = rot_points + np.array([w, h])
    cv2.line(
        I_temp,
        tuple(im_points[0].astype(int)),
        tuple(im_points[1].astype(int)),
        color=(0, 255, 0),
        thickness=5,
    )
    cv2.line(
        I_temp,
        tuple(im_points[1].astype(int)),
        tuple(im_points[2].astype(int)),
        color=(0, 0, 255),
        thickness=5,
    )
    cv2.line(
        I_temp,
        tuple(im_points[2].astype(int)),
        tuple(im_points[3].astype(int)),
        color=(0, 255, 0),
        thickness=5,
    )
    cv2.line(
        I_temp,
        tuple(im_points[3].astype(int)),
        tuple(im_points[0].astype(int)),
        color=(0, 0, 255),
        thickness=5,
    )
    return I_temp


class GraspModel(object):
    """
    This class contains functionality for grasp prediction.
    """

    def __init__(
        self,
        nsamples=128,
        patchsize=50,
        n_importance=1,
        n_sen=0,
        n_sen_samples=0,
        sen_pixels=0,
        sen_metric="mean",
        model_name=None,
        url=None,
    ):
        """
            The constructor for :class:`GraspModel` class.

            :param nsamples: Number of samples from a given image
            :param patchsize: Size of patch to be sampled
            :param n_importance: Number of importance samples
            :param n_sen: Number of patches to perform sensitivity on
            :param n_sen_samples: Number of samples for each patch
            :param sen_pixels: Number of pixels to compute sensitivity over
        :param sen_metric: Metric for sensitivity aggregation
            :param model_name: Name of the downloaded model
        :param url: URL for the model

            :type nsamples: int
            :type patchsize: int
            :type n_importance: int
            :type n_sen: int
            :type n_sen_samples: int
            :type sen_pixels: int
            :type sen_metric: string
            :type model_name: string
            :type url: string
            """
        assert model_name is not None
        assert url is not None
        model_path = os.path.join(SAVE_DIR, model_name)
        download_if_not_present(model_path, url)
        if nsamples > MAX_BATCHSIZE:
            self._batch_size = MAX_BATCHSIZE
            self._nbatches = int(nsamples / MAX_BATCHSIZE) + 1
        else:
            self._batch_size = nsamples
            self._nbatches = 1
        print("Loading grasp model")
        st_time = time.time()
        self.grasp_obj = GraspTorchObj(model_path)
        print("Time taken to load model: {}s".format(time.time() - st_time))
        self.patchsize = patchsize
        self.n_importance = n_importance
        self.n_sen = n_sen
        self.n_sen_samples = n_sen_samples
        self.sen_pixels = sen_pixels
        self.sen_metric = sen_metric
        self.sen_metrics = ["mean", "min"]
        assert self.sen_metric in self.sen_metrics
        assert self.n_sen_samples <= MAX_BATCHSIZE

    def predict(self, I):
        """
        Runs prediction on a given image.

        :param I: An image
        :type I: np.ndarray

        :returns: selected grasp configuration (height, width, angle, confidence)
        :rtype: tuple
        """
        start_time = time.time()
        rospy.loginfo("Running TORCH-GRASPING!")

        # First round of forward pass
        result = self._predict_image(I, self._nbatches, self._batch_size)
        init_predictions, init_patch_Hs, init_patch_Ws = result

        # Predicting for second round of sensitivity
        sen_study = []
        sen_options = []
        if self.n_sen <= 0:
            predictions, patch_Hs, patch_Ws = (
                init_predictions,
                init_patch_Hs,
                init_patch_Ws,
            )
        else:
            im_shape = I.shape
            r = np.argsort(np.max(init_predictions, 1))
            r_best = r[-self.n_sen :][::-1]
            for r_ind in r_best:
                patch_h = init_patch_Hs[r_ind]
                patch_w = init_patch_Ws[r_ind]
                scan_size = self.patchsize + self.sen_pixels
                scan_halfsize = int(scan_size / 2)
                scan_region = [
                    patch_h - scan_halfsize,
                    patch_h + scan_halfsize,
                    patch_w - scan_halfsize,
                    patch_w + scan_halfsize,
                ]
                if (
                    scan_region[0] < 0
                    or scan_region[1] >= im_shape[0]
                    or scan_region[2] < 0
                    or scan_region[3] >= im_shape[1]
                ):
                    continue
                else:
                    I_hw = I[
                        scan_region[0] : scan_region[1], scan_region[2] : scan_region[3]
                    ]
                    result = self._predict_image(I_hw, 1, self.n_sen_samples)
                    hw_predictions, hw_patch_Hs, hw_patch_Ws = result
                    sen_study.append(copy([hw_predictions, hw_patch_Hs, hw_patch_Ws]))
                    sen_options.append(r_ind)
            if len(sen_options) == 0:
                predictions = init_predictions
                patch_Hs = init_patch_Hs
                patch_Ws = init_patch_Ws
            else:
                stability = []
                for option in sen_study:
                    if self.sen_metric == "mean":
                        stability.append(option[0].mean(0))
                    elif self.sen_metric == "min":
                        stability.append(option[0].min(0))
                stability = np.array(stability)
                predictions = copy(stability)
                patch_Hs = init_patch_Hs[sen_options]
                patch_Ws = init_patch_Ws[sen_options]

        # Importance Sampling
        r = np.argsort(np.max(predictions, 1))
        r_best = r[-self.n_importance :][::-1]
        imp_best = np.max(predictions, 1)[r_best]
        imp_best[imp_best < 0] = 0.0
        if imp_best[0] != 0.0:
            imp_best = imp_best / imp_best.sum()
            patch_choice_ind = np.random.choice(self.n_importance, p=imp_best)
            patch_ind = r_best[patch_choice_ind]
        else:
            patch_ind = r_best[0]
        theta_imps = copy(predictions[patch_ind])
        theta_imps[theta_imps < 0] = 0.0
        theta_choice_ind = np.argmax(theta_imps)
        grasp_angle = theta_choice_ind * (np.pi / 18) - np.pi / 2
        selected_grasp = (
            patch_Hs[patch_ind],
            patch_Ws[patch_ind],
            grasp_angle,
            predictions[patch_ind, theta_choice_ind],
        )
        self._disp_I = drawRectangle(
            I,
            selected_grasp[0],
            selected_grasp[1],
            theta_choice_ind,
            int(self.patchsize),
        )
        rospy.loginfo(
            "Grasp prediction took: {} (s)" "".format(time.time() - start_time)
        )
        return selected_grasp

    def _predict_image(self, I, nbs, bs):
        """
        Compute raw network predictions

        :param I: Scene image
        :param nbs: Number of batches
        :param bs: Batch size
        :type I: np.ndarray
        :type nbs: int
        :type bs: int

        :returns: Grasp predictions
        :rtype: tuple
        """
        min_imsize = min(I.shape[:2])
        assert self.patchsize < min_imsize, "Input image dimensions are too small"
        gsize = int(self.patchsize)
        P = Predictors(I, self.grasp_obj)
        predictions = []
        patch_Hs = []
        patch_Ws = []
        rospy.loginfo("Torch grasp_model: Predicting on samples")
        for _ in range(nbs):
            P.graspNet_grasp(patch_size=gsize, num_samples=bs)
            predictions.append(P.norm_vals)
            patch_Hs.append(P.patch_hs)
            patch_Ws.append(P.patch_ws)

        predictions = np.concatenate(predictions)
        patch_Hs = np.concatenate(patch_Hs)
        patch_Ws = np.concatenate(patch_Ws)
        return predictions, patch_Hs, patch_Ws

    def display_predicted_image(self):
        """
        Display the scene image with the predicted grasp
        """
        if self._disp_I is None:
            return False
        print('Visualizing grasp; hit "space" to continue')
        cv2.imshow("image", cv2.cvtColor(self._disp_I, cv2.COLOR_BGR2RGB))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def save_predicted_image(self, img_fname):
        """
        Save predicted grasp on the image

        :param img_fname: Path of image file where the grasp should be saved
        :type img_fname: string
        """
        if self._disp_I is None:
            return
        cv2.imwrite(img_fname, self._disp_I)

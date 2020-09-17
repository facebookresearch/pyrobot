# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
@s-gupta

Code for estimating unknown transforms by setting up a least square pixel
reprojection error problem using pytorch. See README.md for details.
"""

from __future__ import print_function

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import os
import pickle
import numpy as np
import json
import sys
import torch
import torch.optim as optim
from absl import app, flags

FLAGS = flags.FLAGS

flags.DEFINE_string("data_dir", None, "Directory with images and pkl files")
flags.DEFINE_string(
    "calibration_output_file", None, "File where to write calibration parameters"
)
flags.DEFINE_boolean("overwrite", False, "Whether to overwrite output file or not")
flags.DEFINE_float(
    "inlier_pixel_threshold", 20.0, "Max. reprojection error to consider an inlier"
)
flags.DEFINE_float(
    "min_inlier_fraction", 0.2, "Fraction of pts to atleast consider as inliers"
)
flags.DEFINE_integer("n_iters", 10001, "Number of iterations")
flags.DEFINE_string(
    "typ", "camera_arm", "Calibration to run: camera_only | [camera_arm]"
)
flags.DEFINE_list("to_optimize", ["camera_link"], "Static transforms to optimize")
AR_SZ = 0.03 / 2.0


def mkdir_if_missing(output_dir):
    if not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
            return True
        except Exception:
            return False


def load_data(dir_name):
    """
    Load data from the pkl file, and load corresponding images.
    :params dir_name: name of directory
    :type dir_name: string
    :return: data from states.pkl file in directory, images from directory
    :rtype: dict, list of cv2 images (numpy arrays)
    """
    tt = pickle.load(open(os.path.join(dir_name, "states.pkl"), 'rb'))
    imgs = []
    for i in range(tt["states"].shape[0]):
        img = cv2.imread(os.path.join(dir_name, "images", "img_{:04d}.png".format(i)))
        imgs.append(img)
    return tt, imgs


def filter_data(imgs, dt, inds):
    """
    Picks out ind rows from various fields in dt.
    :params imgs: list of cv2 images (numpy arrays)
    :params dt: dictionary with data captured during calibration
    :params ind: numpy array of indices to sample from imgs and dt
    :type imgs: list of cv2 images (numy arrays)
    :type dt: dictionary of transforms
    :type inds: numpy array of indices
    :return: list of images referred by inds, dictionary dt with data sampled
    using indices.
    :rtype: list, dict
    """
    imgs = [imgs[i] for i in inds]
    kk = [
        "rgb_tf_tree_quat",
        "rgb_tf_tree_matrix",
        "states",
        "ar_quat",
        "rgb_tf_tree_trans",
        "ar_trans",
        "arm_pose_ids",
        "ar_tf_tree_quat",
        "ar_tf_tree_trans",
        "ar_img_loc",
    ]
    for k in kk:
        dt[k] = dt[k][inds, ...]
    return imgs, dt


def get_tag_corners(imgs, ar_img_loc, dir_name=None):
    """
    Converts ar_img_loc into useable format, by removing entires that are
    empty. Also, returns good indices and writes out good images into dir_name
    directroy if not None.
    """
    pts = []
    good_imgs = []
    for i, img in enumerate(imgs):
        if not np.any(np.isnan(ar_img_loc[i, ...])):
            pt = ar_img_loc[i, [1, 0, 3, 2], :]
            pts.append(pt)
            good_imgs.append(i)
            if dir_name is not None:
                _img = img * 1
                pt_int = pt.astype(np.int32)
                for j in range(3):
                    cv2.line(
                        _img,
                        (pt_int[j, 0], pt_int[j, 1]),
                        (pt_int[j + 1, 0], pt_int[j + 1, 1]),
                        (255, 0, 0),
                        j + 1,
                    )
                cv2.imwrite(os.path.join(dir_name, "img_{:04d}.jpg".format(i)), _img)

    pts = np.array(pts)
    good_imgs = np.array(good_imgs)
    return pts, good_imgs


def hamilton_product(qa, qb):
    """
    Multiply qa by qb.
    :params qa: B X N X 4 quaternions
    :params qb: B X N X 4 quaternions
    :type qa: torch.Tensor
    :type qb: torch.Tensor
    :return: B X N X 4
    :rtype: torch.Tensor
    """
    qa_0 = qa[:, :, 0]
    qa_1 = qa[:, :, 1]
    qa_2 = qa[:, :, 2]
    qa_3 = qa[:, :, 3]

    qb_0 = qb[:, :, 0]
    qb_1 = qb[:, :, 1]
    qb_2 = qb[:, :, 2]
    qb_3 = qb[:, :, 3]

    # See https://en.wikipedia.org/wiki/Quaternion#Hamilton_product
    q_mult_0 = qa_0 * qb_0 - qa_1 * qb_1 - qa_2 * qb_2 - qa_3 * qb_3
    q_mult_1 = qa_0 * qb_1 + qa_1 * qb_0 + qa_2 * qb_3 - qa_3 * qb_2
    q_mult_2 = qa_0 * qb_2 - qa_1 * qb_3 + qa_2 * qb_0 + qa_3 * qb_1
    q_mult_3 = qa_0 * qb_3 + qa_1 * qb_2 - qa_2 * qb_1 + qa_3 * qb_0

    return torch.stack([q_mult_0, q_mult_1, q_mult_2, q_mult_3], dim=-1)


def quat_rotate(X, q, inverse=False):
    """
    Rotate points by quaternions.
    :params X: B X N X 3 points
    :params q: B X 4 quaternions
    :type X: torch.Tensor
    :type q: torch.Tensor
    :return: B X N X 3 (rotated points)
    :rtype: torch.Tensor
    """
    # repeat q along 2nd dim
    ones_x = X[[0], :, :][:, :, [0]] * 0 + 1
    q = torch.unsqueeze(q, 1) * ones_x

    q_conj = torch.cat([q[:, :, [0]], -1 * q[:, :, 1:4]], dim=-1)
    X = torch.cat([X[:, :, [0]] * 0, X], dim=-1)

    if inverse:
        X_rot = hamilton_product(q_conj, hamilton_product(X, q))
    else:
        X_rot = hamilton_product(q, hamilton_product(X, q_conj))
    return X_rot[:, :, 1:4]


def get_transforms_to_optimize(chain, to_optimize_quat, to_optimize_trans, device):
    """Returns pytorch tensors to optimize along the chain as per
    to_optimize_trans and to_optimize_quat."""
    opt_pyt_trans, opt_pyt_quat = [], []
    for i in range(len(chain) - 1):
        t = None
        q = None
        if chain[i + 1] in to_optimize_trans:
            t = torch.zeros(1, 3, device=device, dtype=torch.double, requires_grad=True)
        if chain[i + 1] in to_optimize_quat:
            qxyz = torch.zeros(
                1, 3, device=device, dtype=torch.double, requires_grad=True
            )
            qw = torch.ones(1, 1, device=device, dtype=torch.double, requires_grad=True)
            q = [qw, qxyz]
        opt_pyt_trans.append(t)
        opt_pyt_quat.append(q)
    return opt_pyt_quat, opt_pyt_trans


def project_pts(pts, quat, trans, opt_pyt_quat, opt_pyt_trans, intrinsics):
    """Projects pts to the camera using the transformation and the camera
    intrinsics."""
    pts_3d = pts
    t_pts_3d = transform_points_from_base(
        pts_3d, quat, trans, opt_pyt_quat, opt_pyt_trans
    )
    t_pts_3d = torch.matmul(intrinsics, t_pts_3d.permute(0, 2, 1)).permute(0, 2, 1)
    t_pts_2d = t_pts_3d[:, :, :2] / t_pts_3d[:, :, 2:]
    return t_pts_2d


def reprojection_error(
    t_pts_2d, pts_2d_observed, min_inlier_fraction, inlier_pixel_threshold, mask=None
):
    """Computes re-projection error for observed and projected points."""
    n_pts = t_pts_2d.shape[0]
    err_all = t_pts_2d - pts_2d_observed
    err_all = err_all ** 2
    err_all = err_all.sum(2)
    topk_k = int(4 * n_pts * min_inlier_fraction)
    topk, _ = torch.topk(err_all.view(-1), k=topk_k, largest=False)
    in_px_thresh_pyt = torch.from_numpy(np.array([inlier_pixel_threshold ** 2]))
    in_px_thresh_pyt = in_px_thresh_pyt.double()
    topk = torch.max(topk[-1], in_px_thresh_pyt)
    err_all_robust = torch.min(topk, err_all)

    if mask is not None:
        err_all_robust = err_all_robust * mask
        err = err_all_robust.sum() / mask.sum()
    else:
        err = err_all_robust.mean()
    err_all = torch.sqrt(err_all)
    return err, err_all, topk


def optimize(
    vars_to_optimize,
    ar_tag_points,
    rgb_quat,
    rgb_trans,
    ar_quat,
    ar_trans,
    rgb_opt_pyt_quat,
    rgb_opt_pyt_trans,
    ar_opt_pyt_quat,
    ar_opt_pyt_trans,
    intrinsics,
    pts_2d_observed,
    optimizer_opts,
    mask=None,
):
    """Optimizes for vars_to_optimize by going through the kinematic chain from
    ar tag points to camera frame, projecting points using the camera
    matrices."""

    optimizer = optim.Adam(vars_to_optimize, lr=0.01)
    print("Beginning optimization.")
    # optimizer = optim.SGD(vars_to_optimize, lr=0.01, momentum=0.9)
    for i in range(optimizer_opts["n_iters"]):
        # zero the parameter gradients
        optimizer.zero_grad()
        t_pts = transform_points_to_base(
            ar_tag_points, ar_quat, ar_trans, ar_opt_pyt_quat, ar_opt_pyt_trans
        )
        t_pts_2d = project_pts(
            t_pts, rgb_quat, rgb_trans, rgb_opt_pyt_quat, rgb_opt_pyt_trans, intrinsics
        )
        min_inlier_fraction = optimizer_opts["min_inlier_fraction"]
        inlier_pixel_threshold = optimizer_opts["inlier_pixel_threshold"]
        err, err_all, robust_err_threshold = reprojection_error(
            t_pts_2d, pts_2d_observed, min_inlier_fraction, inlier_pixel_threshold, mask
        )
        robust_err_threshold = np.sqrt(robust_err_threshold.detach().numpy()[0])
        err.backward()
        optimizer.step()

        if np.mod(i, 1000) == 0 or i == optimizer_opts["n_iters"] - 1:
            np_err_all = err_all.detach().numpy()
            n_inliers = int(np.sum(np_err_all <= robust_err_threshold))
            total_pts = np_err_all.size
            np_err_all = np_err_all[np_err_all <= robust_err_threshold]
            str_ = (
                "Iteration: {:8d}, "
                + "N Inliers: {:d} / {:d}, "
                + "loss: {:0.3f}, "
                + "reprojection error: {:0.3f}, "
                + "inlier px threshold: {:0.3f}, "
            )
            str_ = str_.format(
                i,
                n_inliers,
                total_pts,
                err.detach().numpy(),
                np.mean(np_err_all),
                robust_err_threshold,
            )
            print(str_)
            for v in vars_to_optimize:
                print("  " + str(v.detach().numpy()))
    str_ = "Optimization Finished. Final Loss: {:0.3f}."
    print(str_.format(err.detach().numpy()))
    print("")


def setup_camera_arm_solver(
    dt,
    imgs,
    mask,
    pts_2d,
    to_optimize_trans,
    to_optimize_quat,
    optimizer_opts,
    calibration_output_file=None,
):
    """This version sets up variables for where the base of the arm is with
    respect to the robot base. AR tag corners can then be recovered in terms of
    these variables, and a direct optimization problem can be set up in these
    terms."""

    # Set up variables that need to be optimized.
    device = torch.device("cpu")
    torch.set_num_threads = 4
    torch.manual_seed(4)

    rgb_chain = dt["rgb_chain"]
    ar_chain = dt["ar_chain"]
    rgb_quat = torch.from_numpy(
        dt["rgb_tf_tree_quat"][:, :, [3, 0, 1, 2]].astype(np.float64)
    )
    rgb_trans = torch.from_numpy(dt["rgb_tf_tree_trans"].astype(np.float64))
    ar_quat = torch.from_numpy(
        dt["ar_tf_tree_quat"][:, :, [3, 0, 1, 2]].astype(np.float64)
    )
    ar_trans = torch.from_numpy(dt["ar_tf_tree_trans"].astype(np.float64))
    intrinsics = torch.from_numpy(dt["intrinsics"][:, :3]).double().reshape([1, 3, 3])
    pts_2d_observed = torch.from_numpy(pts_2d).double()

    rgb_opt_pyt_quat, rgb_opt_pyt_trans = get_transforms_to_optimize(
        rgb_chain, to_optimize_quat, to_optimize_trans, device
    )
    ar_opt_pyt_quat, ar_opt_pyt_trans = get_transforms_to_optimize(
        ar_chain, to_optimize_quat, to_optimize_trans, device
    )

    ar_tag_points = np.array(
        [
            [AR_SZ, AR_SZ, 0],
            [AR_SZ, -AR_SZ, 0],
            [-AR_SZ, -AR_SZ, 0],
            [-AR_SZ, AR_SZ, 0],
        ],
        dtype=np.float64,
    )
    ar_tag_points = ar_tag_points[np.newaxis, :, :]
    ar_tag_points = np.tile(ar_tag_points, [ar_quat.shape[0], 1, 1])
    ar_tag_points = torch.from_numpy(ar_tag_points).double()

    # Use gradient descent from pytorch to solve this problem.
    vars_pts = []
    vars_trans = [t for t in rgb_opt_pyt_trans + ar_opt_pyt_trans if t is not None]
    vars_rotate = []
    for q in rgb_opt_pyt_quat + ar_opt_pyt_quat:
        if q is not None:
            vars_rotate.append(q[0])
            vars_rotate.append(q[1])
    if len(vars_pts) > 0:
        optimize(
            vars_pts,
            ar_tag_points,
            rgb_quat,
            rgb_trans,
            ar_quat,
            ar_trans,
            rgb_opt_pyt_quat,
            rgb_opt_pyt_trans,
            ar_opt_pyt_quat,
            ar_opt_pyt_trans,
            intrinsics,
            pts_2d_observed,
            optimizer_opts,
            None,
        )
    if len(vars_pts + vars_trans) > 0:
        optimize(
            vars_pts + vars_trans,
            ar_tag_points,
            rgb_quat,
            rgb_trans,
            ar_quat,
            ar_trans,
            rgb_opt_pyt_quat,
            rgb_opt_pyt_trans,
            ar_opt_pyt_quat,
            ar_opt_pyt_trans,
            intrinsics,
            pts_2d_observed,
            optimizer_opts,
            None,
        )
    if len(vars_pts + vars_trans + vars_rotate) > 0:
        optimize(
            vars_pts + vars_trans + vars_rotate,
            ar_tag_points,
            rgb_quat,
            rgb_trans,
            ar_quat,
            ar_trans,
            rgb_opt_pyt_quat,
            rgb_opt_pyt_trans,
            ar_opt_pyt_quat,
            ar_opt_pyt_trans,
            intrinsics,
            pts_2d_observed,
            optimizer_opts,
            None,
        )

    if calibration_output_file is not None:
        # Prepare a json with the camera calibration parameters, and write them
        # out.
        to_write = to_optimize_trans + to_optimize_quat
        out = {}
        def_trans = np.zeros((3,), dtype=np.float64)
        def_quat = np.zeros((4,), dtype=np.float64)
        def_quat[3] = 1.0
        for t in to_write:
            out[t] = {"trans": def_trans.tolist(), "quat": def_quat.tolist()}
            if t in to_optimize_trans:
                if t in rgb_chain:
                    id_ = rgb_chain.index(t)
                    base_trans = torch.mean(rgb_trans[:, id_ - 1, :], 0, keepdim=True)
                    trans = rgb_opt_pyt_trans[id_ - 1]
                    _from = rgb_chain[id_ - 1]
                    _to = rgb_chain[id_]
                else:
                    id_ = ar_chain.index(t)
                    base_trans = torch.mean(ar_trans[:, id_ - 1, :], 0, keepdim=True)
                    trans = ar_opt_pyt_trans[id_ - 1]
                    _from = ar_chain[id_ - 1]
                    _to = ar_chain[id_]
                out[t]["trans"] = (-trans - base_trans).detach().cpu()
                out[t]["trans"] = out[t]["trans"].numpy()[0, :].tolist()

            if t in to_optimize_quat:
                if t in rgb_chain:
                    id_ = rgb_chain.index(t)
                    base_quat = rgb_quat[:1, id_ - 1, :]
                    quat = rgb_opt_pyt_quat[id_ - 1]
                    _from = rgb_chain[id_ - 1]
                    _to = rgb_chain[id_]
                else:
                    id_ = ar_chain.index(t)
                    base_quat = ar_quat[:1, id_ - 1, :]
                    quat = ar_opt_pyt_quat[id_ - 1]
                    _from = ar_chain[id_ - 1]
                    _to = ar_chain[id_]
                q = torch.cat(quat, 1)
                norm = q.norm(p=2, dim=1, keepdim=True)
                q_normalized = q.div(norm)
                full_quat = hamilton_product(
                    q_normalized.view(1, 1, 4), base_quat.view(1, 1, 4)
                )
                quat = full_quat.detach().cpu().numpy()
                quat = quat[0, 0, [1, 2, 3, 0]]
                # Inverting the quat
                quat[:3] = -quat[:3]
                quat = quat.tolist()
                out[t]["quat"] = quat
            out[t]["from"] = _from
            out[t]["to"] = _to
        print("Writing calibration parameters to {:s}.".format(calibration_output_file))
        with open(calibration_output_file, "w") as f:
            json.dump(out, f, sort_keys=True, indent=4, separators=(",", ": "))


def setup_camera_solver(
    dt, imgs, mask, pts_2d, to_optimize_trans=[], to_optimize_quat=[]
):
    # TODO(s-gupta): Clean up this function, so that it uses the common
    # functions that we have defined above.
    n_measures = pts_2d.shape[0]
    quat = dt["rgb_tf_tree_quat"] * 1.0
    quat = quat[:, :, [3, 0, 1, 2]]
    trans = dt["rgb_tf_tree_trans"] * 1.0
    intrinsics = dt["intrinsics"]
    chain = dt["rgb_chain"]

    init_pts = get_ar_tag_loc(
        dt["ar_tf_tree_quat"], dt["ar_tf_tree_trans"], dt["arm_pose_ids"]
    )
    init = init_pts.T
    init = init[np.newaxis, :, :].astype(np.float64)
    init = np.repeat(init, 4, 2)

    torch.manual_seed(4)
    # Set up variables that will be optimized.
    device = torch.device("cpu")
    # 3D Points
    _pts_3d = torch.from_numpy(init)
    _pts_3d.requires_grad = True

    # Translation and rotations as needed.
    opt_pyt_trans, opt_pyt_quat = [], []
    for i in range(trans.shape[1]):
        t = None
        q = None
        if chain[i + 1] in to_optimize_trans:
            t = torch.zeros(1, 3, device=device, dtype=torch.double, requires_grad=True)
        if chain[i + 1] in to_optimize_quat:
            qxyz = torch.zeros(
                1, 3, device=device, dtype=torch.double, requires_grad=True
            )
            qw = torch.ones(1, 1, device=device, dtype=torch.double, requires_grad=True)
            q = [qw, qxyz]
        opt_pyt_trans.append(t)
        opt_pyt_quat.append(q)

    # Load data into pytorch
    pyt_quat = torch.from_numpy(quat)
    pyt_trans = torch.from_numpy(trans)
    pts_2d_observed = torch.from_numpy(np.transpose(pts_2d, [0, 2, 1])).double()
    pyt_mask = torch.from_numpy(mask.astype(np.float64))
    intrinsics_pyt = torch.from_numpy(intrinsics[:, :3]).double().reshape([1, 3, 3])

    def project_pts(
        _pts, pyt_quat, pyt_trans, opt_pyt_quat, opt_pyt_trans, intrinsics_pyt
    ):
        pts_3d = _pts_3d
        pts_3d = pts_3d.permute(0, 2, 1)

        t_pts_3d = pts_3d.view(1, -1, 3).repeat(n_measures, 1, 1)
        t_pts_3d = transform_points_from_base(
            pts_3d, pyt_quat, pyt_trans, opt_pyt_quat, opt_pyt_trans
        )

        t_pts_3d = t_pts_3d.permute(0, 2, 1)
        t_pts_3d = torch.matmul(intrinsics_pyt, t_pts_3d)
        t_pts_2d = t_pts_3d[:, :2, :] / t_pts_3d[:, 2:, :]
        return t_pts_2d

    def criterion(pyt_mask, t_pts_2d, pts_2d_observed):
        err = t_pts_2d - pts_2d_observed
        err = err ** 2
        err = err.sum(1)
        err_all = err * pyt_mask
        # n_pts = t_pts_2d.shape[0]
        # topk, _ = torch.topk(err_all.view(-1), k=int(4*n_pts*0.8),
        # largest=False)
        # mask_robust = torch.le(err_all, topk[-1]).double();
        # err_all = err_all * mask_robust
        # pyt_mask = pyt_mask * mask_robust
        err = err_all.sum() / pyt_mask.sum()
        return err, err_all

    def optimize(
        n_iters,
        vars_to_optimize,
        _pts,
        pyt_quat,
        pyt_trans,
        opt_pyt_quat,
        opt_pyt_trans,
        intrinsics_pyt,
        mask,
        pts_2d_observed,
    ):
        optimizer = optim.Adam(vars_to_optimize, lr=0.01)
        for i in range(n_iters):
            # zero the parameter gradients
            optimizer.zero_grad()
            t_pts_2d = project_pts(
                _pts, pyt_quat, pyt_trans, opt_pyt_quat, opt_pyt_trans, intrinsics_pyt
            )
            err, err_all = criterion(pyt_mask, t_pts_2d, pts_2d_observed)
            err.backward()
            optimizer.step()
            if np.mod(i, 1000) == 0:
                print(err.detach().numpy())
                print(err_all.detach().numpy())
                for v in vars_to_optimize:
                    print(v.detach().numpy())
        print(err.detach().numpy())

    # Use gradient descent from pytorch to solve this problem.
    vars_pts = [_pts_3d]
    vars_trans = [_ for _ in opt_pyt_trans if _ is not None]
    vars_rotate = []
    for q in opt_pyt_quat:
        if q is not None:
            vars_rotate.append(q[0])
            vars_rotate.append(q[1])
    n_iters = 10001
    optimize(
        n_iters,
        vars_pts,
        _pts_3d,
        pyt_quat,
        pyt_trans,
        opt_pyt_quat,
        opt_pyt_trans,
        intrinsics_pyt,
        mask,
        pts_2d_observed,
    )
    optimize(
        n_iters,
        vars_trans + vars_pts,
        _pts_3d,
        pyt_quat,
        pyt_trans,
        opt_pyt_quat,
        opt_pyt_trans,
        intrinsics_pyt,
        mask,
        pts_2d_observed,
    )
    optimize(
        n_iters,
        vars_rotate + vars_trans + vars_pts,
        _pts_3d,
        pyt_quat,
        pyt_trans,
        opt_pyt_quat,
        opt_pyt_trans,
        intrinsics_pyt,
        mask,
        pts_2d_observed,
    )


def transform_points_to_base(pts, quat, trans, opt_quat, opt_trans):
    """Transform pts from target frame to base frame.
    pts is B x N x 3 """
    n_measures = pts.shape[0]
    for i in range(trans.shape[1] - 1, -1, -1):
        if opt_trans[i] is not None:
            pts = pts - opt_trans[i].unsqueeze(1)
        pts = pts - trans[:, i, :].unsqueeze(1)

        if opt_quat[i] is not None:
            q = torch.cat(opt_quat[i], 1)
            norm = q.norm(p=2, dim=1, keepdim=True)
            q_normalized = q.div(norm).repeat(n_measures, 1)
            pts = quat_rotate(pts, q_normalized, inverse=True)
        pts = quat_rotate(pts, quat[:, i, :], inverse=True)
    return pts


def transform_points_from_base(pts, quat, trans, opt_quat, opt_trans):
    """Transform pts from base frame to target frame.
    pts is B x N x 3 """
    t_pts_3d = pts
    n_measures = pts.shape[0]
    for j in range(quat.shape[1]):
        t_pts_3d = quat_rotate(t_pts_3d, quat[:, j, :])
        if opt_quat[j] is not None:
            q = torch.cat(opt_quat[j], 1)
            norm = q.norm(p=2, dim=1, keepdim=True)
            q_normalized = q.div(norm).repeat(n_measures, 1)
            t_pts_3d = quat_rotate(t_pts_3d, q_normalized)

        t_pts_3d = t_pts_3d + trans[:, j, :].unsqueeze(1)
        if opt_trans[j] is not None:
            t_pts_3d = t_pts_3d + opt_trans[j].unsqueeze(1)
    return t_pts_3d


def get_ar_tag_loc(ar_tf_tree_quat, ar_tf_tree_trans, arm_pose_ids):
    """Returns the 3D coordinate of the center of the AR tag based on the
    kinematic chain. Returns one center per unique arm_pose_ids."""
    quat = ar_tf_tree_quat.astype(np.float64)
    trans = ar_tf_tree_trans.astype(np.float64)
    u_pts, ids = np.unique(arm_pose_ids, return_inverse=True)
    pyt_quat = torch.from_numpy(quat[:, :, [3, 0, 1, 2]])
    pyt_trans = torch.from_numpy(trans)
    pts = np.zeros((quat.shape[0], 1, 3), dtype=np.float64)
    opt_pyt_quat = [None for i in range(pyt_trans.shape[1])]
    opt_pyt_trans = [None for i in range(pyt_trans.shape[1])]
    t_pts = transform_points_to_base(
        pts, pyt_quat, pyt_trans, opt_pyt_quat, opt_pyt_trans
    )
    t_pts = t_pts.detach().numpy()
    pts_out = []
    for i in range(u_pts.size):
        pts_out.append(np.mean(t_pts[ids == i, 0, :], 0))
    pts_out = np.array(pts_out)
    return pts_out


def get_correspondence(arm_pose_ids, pts):
    """Use the arm_pose_ids to determine which corners actually corresspond to
    one another. Returns an expanded matrix, and a mask that determines which
    points should be used for computing the loss."""
    u_pts, ids = np.unique(arm_pose_ids, return_inverse=True)
    n_pts = u_pts.size * 4
    mask = np.zeros((pts.shape[0], n_pts), dtype=np.bool)
    pts_full = np.zeros((pts.shape[0], n_pts, 2), dtype=np.float32)
    for i in range(u_pts.size):
        pts_full[ids == i, i * 4 : i * 4 + 4, :] = pts[ids == i, :, :]
        if np.sum(ids == i) >= 3:
            mask[ids == i, i * 4 : i * 4 + 4] = True
    return mask, pts_full


def main(_):
    if FLAGS.calibration_output_file is None:
        FLAGS.calibration_output_file = os.path.join(FLAGS.data_dir, "calibrated.json")
    if os.path.exists(FLAGS.calibration_output_file) and not FLAGS.overwrite:
        print(
            """Calibration file already exists, please specify --overwrite to
            overwrite it with new calibration parameters"""
        )
        sys.exit()

    optimizer_opts = {
        "min_inlier_fraction": FLAGS.min_inlier_fraction,
        "inlier_pixel_threshold": FLAGS.inlier_pixel_threshold,
        "n_iters": FLAGS.n_iters,
    }
    print("optimizer_opts: ", optimizer_opts)
    print("")
    print("transforms to otpimize: ", FLAGS.to_optimize)
    print("")
    dir_name = FLAGS.data_dir

    tag_detect_dir_name = os.path.join(dir_name, "tag_detect")
    mkdir_if_missing(tag_detect_dir_name)
    dt, imgs = load_data(dir_name)
    dt["ar_img_loc"] = np.array(dt["ar_img_loc"])
    ar_img_loc = dt["ar_img_loc"]
    pts, inds = get_tag_corners(imgs, ar_img_loc, tag_detect_dir_name)

    # Remove ones that did not get detected.
    imgs, dt = filter_data(imgs, dt, inds)

    if FLAGS.typ == "camera_only":
        # Set up correspondence between them based on the pose id for the arm.
        mask, pts = get_correspondence(dt["arm_pose_ids"], pts)
        to_optimize_trans = FLAGS.to_optimize
        to_optimize_quat = FLAGS.to_optimize
        calibration_output_file = FLAGS.calibration_output_file
        setup_camera_solver(
            dt,
            imgs,
            mask,
            pts,
            to_optimize_trans,
            to_optimize_quat,
            calibration_output_file=calibration_output_file,
        )
    else:
        mask = []
        to_optimize = FLAGS.to_optimize
        to_optimize_trans = to_optimize
        to_optimize_quat = to_optimize
        calibration_output_file = FLAGS.calibration_output_file
        setup_camera_arm_solver(
            dt,
            imgs,
            mask,
            pts,
            to_optimize_trans,
            to_optimize_quat,
            optimizer_opts,
            calibration_output_file,
        )


if __name__ == "__main__":
    app.run(main)

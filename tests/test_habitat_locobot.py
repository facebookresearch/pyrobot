# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest
import os
import sys
import numpy as np
from quaternion.numpy_quaternion import quaternion

from pyrobot import Robot

from pyrobot.locobot.base_control_utils import _get_absolute_pose
from pyrobot.locobot.bicycle_model import wrap_theta
from habitat_sim import AgentState, SixDOFPose


@pytest.fixture(scope="module")
def create_robot():
    # Please change this to match your habitat_sim repo's path
    path_to_habitat_scene = os.path.dirname(os.path.realpath(__file__))
    relative_path = "../examples/habitat/scenes/skokloster-castle.glb"

    common_config = dict(scene_path=os.path.join(path_to_habitat_scene, relative_path))
    bot = Robot("habitat", common_config=common_config)

    return bot


def test_camera_reset(create_robot):
    bot = create_robot
    bot.camera.reset()
    assert np.allclose(bot.camera.get_state(), [0.0, 0.0], rtol=1e-3)


@pytest.mark.parametrize(
    "target_position", [[0, 0.7], [0.4, 0.4], [0.4, -0.4], [-0.4, 0.4], [-0.4, -0.4]]
)
def test_camera_position_control(create_robot, target_position):
    thr = 1e-3
    bot = create_robot
    bot.camera.reset()
    bot.camera.set_pan_tilt(target_position[0], target_position[1], wait=True)
    assert np.allclose(bot.camera.get_state(), target_position, rtol=thr)
    assert np.allclose(bot.camera.state, target_position, rtol=thr)
    assert np.allclose([bot.camera.get_pan(), bot.camera.get_tilt()], np.array(target_position), rtol=thr)

    bot.camera.reset()
    bot.camera.set_pan(target_position[0], wait=True)
    bot.camera.set_tilt(target_position[1], wait=True)
    assert np.allclose(bot.camera.get_state(), target_position, rtol=thr)
    assert np.allclose(bot.camera.state, target_position, rtol=thr)
    assert np.allclose([bot.camera.get_pan(), bot.camera.get_tilt()], np.array(target_position), rtol=thr)


def test_get_images(create_robot):
    bot = create_robot
    rgb_img = bot.camera.get_rgb()
    depth_img = bot.camera.get_depth()
    rgb_sensor, depth_sensor = bot.camera.agent.agent_config.sensor_specifications
    assert depth_img is not None and depth_img.shape == tuple(depth_sensor.resolution)
    assert rgb_img is not None and rgb_img.shape[:2] == tuple(rgb_sensor.resolution[:2]) \
           and rgb_img.shape[2] == 3
    rgb_img, depth_img = bot.camera.get_rgb_depth()
    assert depth_img is not None and depth_img.shape == tuple(depth_sensor.resolution)
    assert rgb_img is not None and rgb_img.shape[:2] == tuple(rgb_sensor.resolution[:2]) \
           and rgb_img.shape[2] == 3


def test_camera_matrix(create_robot):
    thr = 1e-3
    bot = create_robot
    rgb_sensor, depth_sensor = bot.camera.agent.agent_config.sensor_specifications
    xc, yc = rgb_sensor.resolution[0] / 2.0, rgb_sensor.resolution[1] / 2.0,
    f = (rgb_sensor.resolution[0] / 2.) / np.tan(np.deg2rad(float(rgb_sensor.parameters['hfov']) / 2.))
    camera_mat = np.array([[f, 0, xc],
                           [0, f, yc],
                           [0, 0, 1]])
    assert np.allclose(camera_mat, bot.camera.get_intrinsics(), rtol=thr)


def test_pix_to_3dpt(create_robot):
    bot = create_robot
    pos_thr = 1e-2
    pix_thr = 3
    # set the agent and sensor state
    state = AgentState(position=np.array([-1.1051195, 0.12259939, 18.529133], dtype=np.float32),
                       rotation=quaternion(-1, 0, 0, 0), velocity=np.array([0., 0., 0.]),
                       angular_velocity=np.array([0., 0., 0.]), force=np.array([0., 0., 0.]),
                       torque=np.array([0., 0., 0.]),
                       sensor_states={'rgb': SixDOFPose(
                           position=np.array([-1.1051195, 0.7225994, 18.529133], dtype=np.float32),
                           rotation=quaternion(-1, 0, 0, 0)),
                           'depth': SixDOFPose(position=np.array([-1.1051195, 0.7225994, 18.529133],
                                                                 dtype=np.float32),
                                               rotation=quaternion(-1, 0, 0, 0))})
    bot.base.agent.set_state(state)

    r = [371, 207, 34, 506, 218, 151, 145, 489, 458, 1]
    c = [200, 384, 59, 153, 19, 427, 180, 164, 131, 489]
    gt_loc_base = np.array([[1.54176974, 0.33726213, -0.09259185],
                            [14.11524773, -7.05762386, 3.30174666],
                            [6.51043224, 5.00998106, 6.24576548],
                            [0.71574652, 0.28797614, -0.09897119],
                            [6.69008064, 6.19355122, 1.59305887],
                            [11.33760262, -7.57316425, 5.2501886],
                            [13.15357399, 3.90496728, 6.3033075],
                            [0.76817065, 0.27606133, -0.0991553],
                            [0.88765943, 0.43342746, -0.10041875],
                            [5.63283014, -5.12675556, 6.21082692]])
    gt_loc_cam = np.array([[-0.33726213, 0.69259188, 1.54176974],
                           [7.05762386, -2.70174664, 14.11524773],
                           [-5.00998106, -5.64576546, 6.51043224],
                           [-0.28797614, 0.69897121, 0.71574652],
                           [-6.19355122, -0.99305885, 6.69008064],
                           [7.57316425, -4.65018857, 11.33760262],
                           [-3.90496728, -5.70330747, 13.15357399],
                           [-0.27606133, 0.69915532, 0.76817065],
                           [-0.43342746, 0.70041877, 0.88765943],
                           [5.12675556, -5.6108269, 5.63283014]])
    gt_color = np.array([[129, 113, 100],
                         [89, 72, 69],
                         [154, 126, 99],
                         [150, 119, 99],
                         [135, 123, 121],
                         [91, 74, 67],
                         [88, 73, 67],
                         [158, 127, 107],
                         [144, 115, 94],
                         [87, 71, 63]], dtype=np.uint8)
    loc_in_base, color_in_base = bot.camera.pix_to_3dpt(r, c)
    loc_in_cam, color_in_cam = bot.camera.pix_to_3dpt(r, c, in_cam=True)
    assert np.allclose(gt_loc_base, loc_in_base, rtol=pos_thr) and \
           np.allclose(gt_color, color_in_base, rtol=pix_thr)
    assert np.allclose(gt_loc_cam, loc_in_cam, rtol=pos_thr) and \
           np.allclose(gt_color, color_in_cam, rtol=pix_thr)


def test_pcd(create_robot):
    bot = create_robot
    pos_thr = 1e-2
    pix_thr = 3
    # set the agent and sensor state
    state = AgentState(position=np.array([-1.1051195, 0.12259939, 18.529133], dtype=np.float32),
                       rotation=quaternion(-1, 0, 0, 0), velocity=np.array([0., 0., 0.]),
                       angular_velocity=np.array([0., 0., 0.]), force=np.array([0., 0., 0.]),
                       torque=np.array([0., 0., 0.]),
                       sensor_states={'rgb': SixDOFPose(
                           position=np.array([-1.1051195, 0.7225994, 18.529133], dtype=np.float32),
                           rotation=quaternion(-1, 0, 0, 0)),
                           'depth': SixDOFPose(position=np.array([-1.1051195, 0.7225994, 18.529133],
                                                                 dtype=np.float32),
                                               rotation=quaternion(-1, 0, 0, 0))})
    bot.base.agent.set_state(state)

    pts_in_cam, color_in_cam = bot.camera.get_current_pcd(in_cam=True)
    pts_in_base, color_in_base = bot.camera.get_current_pcd(in_cam=False)

    # check on randomly sampled 10 points data points collected by visualizing the pointcloud
    gt_indx = [239969, 45352, 52255, 8058, 59402, 115463, 253441, 66933, 211776, 158488]
    gt_pcd_in_base = np.array([[0.83611417, -0.37233209, -0.09240702],
                               [7.54853964, -1.17945932, 5.55372916],
                               [7.61673594, 6.69439682, 5.18194273],
                               [5.8921771, -2.80799065, 6.14693238],
                               [6.94428158, 6.67302058, 4.39765401],
                               [3.42053533, -0.32067519, 1.01420547],
                               [0.74898368, 0.69632077, -0.09924646],
                               [10.76923084, -4.92187503, 5.90048083],
                               [1.1289084, -0.35719367, -0.09233833],
                               [3.29990649, -0.52850065, -0.08318374]])
    gt_pcd_in_cam = np.array([[0.37233209, 0.69240705, 0.83611417],
                              [1.17945932, -4.95372914, 7.54853964],
                              [-6.69439682, -4.58194271, 7.61673594],
                              [2.80799065, -5.54693235, 5.8921771],
                              [-6.67302058, -3.79765399, 6.94428158],
                              [0.32067519, -0.41420545, 3.42053533],
                              [-0.69632077, 0.69924648, 0.74898368],
                              [4.92187503, -5.30048081, 10.76923084],
                              [0.35719367, 0.69233835, 1.1289084],
                              [0.52850065, 0.68318377, 3.29990649]])
    gt_color = np.array([[127, 113, 100],
                         [159, 125, 100],
                         [94, 86, 75],
                         [182, 155, 140],
                         [64, 45, 36],
                         [91, 86, 89],
                         [141, 124, 112],
                         [119, 86, 67],
                         [131, 117, 104],
                         [154, 131, 104]], dtype=np.uint8)
    assert np.allclose(gt_pcd_in_base, pts_in_base[gt_indx],  rtol=pos_thr) and \
           np.allclose(gt_color, color_in_base[gt_indx], rtol=pix_thr)
    assert np.allclose(gt_pcd_in_cam, pts_in_cam[gt_indx], rtol=pos_thr) and \
           np.allclose(gt_color, color_in_cam[gt_indx], rtol=pix_thr)


posns = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, np.pi],
        [1.0, 1.0, np.pi / 2.0],
        [1.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ],
    dtype=np.float32,
)

trans_thresh = 0.01
angular_thresh = np.deg2rad(1)


@pytest.mark.parametrize("posn", posns)
def test_absolute_position_control(
        create_robot, posn,
):
    bot = create_robot
    bot.base.go_to_absolute(posn)
    end_state = np.array(bot.base.get_state("odom"))

    dist = np.linalg.norm(end_state[:2] - posn[:2])
    angle = end_state[2] - posn[2]
    angle = np.abs(wrap_theta(angle))
    assert dist < trans_thresh
    assert angle < angular_thresh


@pytest.mark.parametrize("posn", posns)
def test_relative_position_control(
        create_robot, posn,
):
    bot = create_robot
    start_state = np.array(bot.base.get_state("odom"))
    desired_target = _get_absolute_pose(posn, start_state)
    bot.base.go_to_relative(posn)
    end_state = np.array(bot.base.get_state("odom"))

    dist = np.linalg.norm(end_state[:2] - desired_target[:2])
    angle = end_state[2] - desired_target[2]
    angle = np.abs(wrap_theta(angle))
    assert dist < trans_thresh
    assert angle < angular_thresh

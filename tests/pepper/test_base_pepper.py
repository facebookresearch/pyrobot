import pytest
import time
import rospy
import numpy as np
from pyrobot import Robot


@pytest.fixture
def virtualenv(request):
    return request.config.getoption('virtualenv')


@pytest.fixture(scope="module")
def create_robot():
    return Robot('pepper', use_arm=False, use_camera=False, use_gripper=False)


def test_base_topics(create_robot):
    bot = create_robot
    topic_names = [x[0] for x in rospy.get_published_topics()]
    assert bot.configs.BASE.ROSTOPIC_BASE_COMMAND in topic_names
    assert bot.configs.BASE.ROSTOPIC_BASE_POS_COMMAND in topic_names
    assert bot.configs.BASE.ROSTOPIC_ODOMETRY in topic_names


@pytest.mark.parametrize("exe_time", [0.3, 2.0])
def test_set_vel(create_robot, exe_time):
    epsilon_seconds = 0.2
    bot = create_robot

    # Test set_vel with a turn speed
    start_time = time.time()
    bot.base.set_vel(0.0, 0.0, 0.3, exe_time=exe_time)
    assert (time.time() - start_time) < exe_time + epsilon_seconds

    # Test set_vel with a forward speed
    start_time = time.time()
    bot.base.set_vel(0.2, 0.0, 0.0, exe_time=exe_time)
    assert (time.time() - start_time) < exe_time + epsilon_seconds

    # Test set_vel with a forward  and a turn speed
    start_time = time.time()
    bot.base.set_vel(0.1, 0.1, 0.2, exe_time=exe_time)
    assert (time.time() - start_time) < exe_time + epsilon_seconds


def test_go_to_relative(create_robot):
    xyt_position = [[0.8, 0.7, 0.0], [-0.8, -0.7, 0.0]]
    bot = create_robot

    start_pose = bot.base.get_state()
    bot.base.go_to_relative(xyt_position[0])
    bot.base.go_to_relative(xyt_position[1])
    end_pose = bot.base.get_state()

    assert abs(end_pose[2] - start_pose[2]) < 0.15
    assert np.linalg.norm(
        np.array(end_pose[:2]) - np.array(start_pose[:2])) < 0.2


@pytest.mark.parametrize("xyt_position", [[0.1, 0.3, 0.5], [-1.0, -0.2, 0.0]])
def test_go_to_absolute(create_robot, xyt_position):
    bot = create_robot

    bot.base.go_to_absolute(xyt_position)
    assert abs(xyt_position[2] - bot.base.get_state()[2]) < 0.15
    assert np.linalg.norm(
        np.array(xyt_position[:2]) - np.array(bot.base.get_state()[:2])) < 0.2

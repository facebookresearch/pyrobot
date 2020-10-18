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
    return Robot('pepper', use_camera=False, use_gripper=False, use_base=False)


def test_arm_topics(create_robot):
    bot = create_robot
    topic_names = [x[0] for x in rospy.get_published_topics()]
    assert bot.configs.ARM.ROSTOPIC_JOINT_STATES in topic_names
    assert bot.configs.ARM.ROSTOPIC_SET_JOINT in topic_names


@pytest.mark.parametrize("r_arm", [True, False])
def test_go_home(create_robot, r_arm):
    bot = create_robot
    epsilon_seconds = 0.2
    epsilon_angles = bot.configs.ARM.MAX_JOINT_ERROR

    # Assert that wait=True ensures an asynchronous call
    bot.arm.move_to_neutral(r_arm=r_arm, wait=True)
    start_time = time.time()
    bot.arm.go_home(r_arm=r_arm, wait=False)
    assert (time.time() - start_time) < epsilon_seconds

    bot.arm.move_to_neutral(r_arm=r_arm, wait=True)
    home_pose = np.zeros(bot.arm.arm_dof)
    bot.arm.go_home(r_arm=r_arm, wait=True)

    if r_arm:
        assert np.linalg.norm(
            bot.arm.get_joint_angles() - home_pose) < epsilon_angles
    else:
        assert np.linalg.norm(
            bot.arm.get_extra_joint_angles() - home_pose) < epsilon_angles


@pytest.mark.parametrize("r_arm", [True, False])
def test_move_to_neutral(create_robot, r_arm):
    bot = create_robot
    epsilon_seconds = 0.2
    epsilon_angles = bot.configs.ARM.MAX_JOINT_ERROR

    # Assert that wait=True ensures an asynchronous call
    bot.arm.go_home(r_arm=r_arm, wait=True)
    start_time = time.time()
    bot.arm.move_to_neutral(r_arm=r_arm, wait=False)
    assert (time.time() - start_time) < epsilon_seconds

    bot.arm.go_home(r_arm=r_arm, wait=True)
    bot.arm.move_to_neutral(r_arm=r_arm, wait=True)

    if r_arm:
        neutral_pose = np.array([1.580, -0.115, 1.226, 0.518, 0.027])
        assert np.linalg.norm(
            bot.arm.get_joint_angles() - neutral_pose) < epsilon_angles
    else:
        neutral_pose = np.array([1.580, 0.115, -1.226, -0.518, -0.027])
        assert np.linalg.norm(
            bot.arm.get_extra_joint_angles() - neutral_pose) < epsilon_angles


def test_ee_poses(create_robot):
    ee_delta = 0.05
    bot = create_robot

    ee_get = bot.arm.get_ee_pose(bot.configs.ARM.ARM_BASE_FRAME)
    extra_ee_get = bot.arm.get_extra_ee_pose(bot.configs.ARM.ARM_BASE_FRAME)

    assert np.linalg.norm(bot.arm.pose_ee[0] - ee_get[0]) < ee_delta
    assert np.linalg.norm(
        bot.arm.pose_extra_ee[0] - extra_ee_get[0]) < ee_delta


def test_get_joint_velocities(create_robot):
    bot = create_robot

    with pytest.raises(NotImplementedError):
        bot.arm.get_joint_velocities()


def test_get_joint_torques(create_robot):
    bot = create_robot

    with pytest.raises(NotImplementedError):
        bot.arm.get_joint_torques()


@pytest.mark.parametrize("r_arm", [True, False])
def test_set_joint_positions(create_robot, r_arm):
    bot = create_robot

    if r_arm:
        positions = [1.580, -0.115, 1.226, 0.518, 0.027]
    else:
        positions = [1.580, 0.115, -1.226, -0.518, -0.027]

    print(bot.arm.go_home(r_arm=r_arm, wait=True))

    assert bot.arm.set_joint_positions(
        positions,
        r_arm=r_arm,
        plan=False,
        wait=True)

    # if wait:
    #     assert success
    # else:
    #     assert not success
    #     time.sleep(3.0)

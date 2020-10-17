import pytest
import rospy
from pyrobot import Robot


@pytest.fixture
def virtualenv(request):
    return request.config.getoption('virtualenv')


@pytest.fixture(scope="module")
def create_robot():
    return Robot('pepper', use_camera=False, use_base=False)


def test_gripper_topics(create_robot):
    bot = create_robot
    topic_names = [x[0] for x in rospy.get_published_topics()]
    assert bot.configs.GRIPPER.ROSTOPIC_SET_JOINT in topic_names


def test_gripper_opened_closed_positions(create_robot):
    bot = create_robot
    assert bot.configs.GRIPPER.GRIPPER_OPENED_POSITION == 1.0
    assert bot.configs.GRIPPER.GRIPPER_CLOSED_POSITION == 0.0


@pytest.mark.parametrize("angle", [-0.3, 0.5, 0.6, 1.5])
@pytest.mark.parametrize("r_hand", [True, False])
@pytest.mark.parametrize("speed", [-0.3, 0.5, 1.5])
def test_gripper_set_angle(create_robot, angle, r_hand, speed):
    bot = create_robot

    try:
        bot.gripper.set_angle(angle, r_hand=r_hand, speed=speed)

    except Exception as e:
        pytest.fail("Unexpected Exception: " + str(e))

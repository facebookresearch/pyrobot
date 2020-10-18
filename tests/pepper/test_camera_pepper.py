import pytest
import rospy
from pyrobot import Robot


@pytest.fixture
def virtualenv(request):
    return request.config.getoption('virtualenv')


@pytest.fixture(scope="module")
def create_robot():
    return Robot('pepper', use_base=False, use_arm=False, use_gripper=False)


def test_camera_topics(create_robot):
    bot = create_robot
    topic_names = [x[0] for x in rospy.get_published_topics()]
    assert bot.configs.CAMERA.ROSTOPIC_CAMERA_TOP_STREAM in topic_names
    assert bot.configs.CAMERA.ROSTOPIC_CAMERA_TOP_INFO_STREAM in topic_names
    assert bot.configs.CAMERA.ROSTOPIC_CAMERA_BOTTOM_STREAM in topic_names
    assert bot.configs.CAMERA.ROSTOPIC_CAMERA_DEPTH_STREAM in topic_names
    assert bot.configs.CAMERA.ROSTOPIC_CAMERA_BOTTOM_INFO_STREAM in topic_names
    assert bot.configs.CAMERA.ROSTOPIC_CAMERA_DEPTH_INFO_STREAM in topic_names


def test_get_rgb(create_robot):
    bot = create_robot

    with pytest.raises(NotImplementedError):
        bot.camera.get_rgb()


def test_get_rgb_top(create_robot):
    bot = create_robot
    top_image = bot.camera.get_rgb_top()

    # Assert that the top RGB image is not None and that the image has 3
    # channels
    assert top_image is not None
    assert top_image.shape[2] == 3


def test_get_rgb_bottom(create_robot):
    bot = create_robot
    bottom_image = bot.camera.get_rgb_bottom()

    assert bottom_image is not None
    assert bottom_image.shape[2] == 3


def test_get_depth(create_robot):
    bot = create_robot
    depth_image = bot.camera.get_depth()

    assert depth_image is not None
    assert len(depth_image.shape) == 2


def test_get_intrinsics(create_robot):
    bot = create_robot
    projection = bot.camera.get_intrinsics()
    assert projection.shape == (3, 3)

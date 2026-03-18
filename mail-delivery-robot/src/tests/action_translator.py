import sys
import types
from pathlib import Path

import pytest

SRC_DIR = Path(__file__).resolve().parents[1]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


# Mock
class FakeVector:
    def __init__(self):
        self.x = 0.0


class FakeTwist:
    def __init__(self):
        self.linear = FakeVector()
        self.angular = FakeVector()


fake_geometry_msgs = types.ModuleType("geometry_msgs")
fake_geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
fake_geometry_msgs.msg.Twist = FakeTwist
sys.modules["geometry_msgs"] = fake_geometry_msgs
sys.modules["geometry_msgs.msg"] = fake_geometry_msgs.msg

# -------------------------------------------------

from control.action_translator import ActionTranslator


@pytest.fixture
def mock_config(monkeypatch):
    fake_config = {
        "GO_MSG_LIN_SPEED": 0.5,
        "BACK_MSG_LIN_SPEED": -0.4,
        "LEFT_TURN_ANG_SPEED": 1.2,
        "RIGHT_TURN_ANG_SPEED": -1.2,
        "U_TURN_ANG_SPEED": 2.5,
    }

    monkeypatch.setattr("control.action_translator.loadConfig", lambda: fake_config)
    return fake_config


@pytest.fixture
def translator(mock_config):
    return ActionTranslator()


def assert_twist(twist, lin_x, ang_z):
    assert isinstance(twist, FakeTwist)
    assert twist.linear.x == pytest.approx(lin_x)
    assert twist.angular.z == pytest.approx(ang_z)


# Action tests
def test_go_action(translator, mock_config):
    twist = translator.translate_action("GO")
    assert_twist(twist, mock_config["GO_MSG_LIN_SPEED"], 0.0)


def test_wait_action(translator):
    twist = translator.translate_action("WAIT")
    assert_twist(twist, 0.0, 0.0)


def test_back_action(translator, mock_config):
    twist = translator.translate_action("BACK")
    assert_twist(twist, mock_config["BACK_MSG_LIN_SPEED"], 0.0)


def test_left_turn_action(translator, mock_config):
    twist = translator.translate_action("LEFT_TURN")
    assert_twist(twist, 0.0, mock_config["LEFT_TURN_ANG_SPEED"])


def test_right_turn_action(translator, mock_config):
    twist = translator.translate_action("RIGHT_TURN")
    assert_twist(twist, 0.0, mock_config["RIGHT_TURN_ANG_SPEED"])


def test_u_turn_action(translator, mock_config):
    twist = translator.translate_action("U_TURN")
    assert_twist(twist, 0.0, mock_config["U_TURN_ANG_SPEED"])


# WALL_FOLLOW parsing tests
def test_wall_follow_valid(translator):
    twist = translator.translate_action("3,0.25,-0.75")
    assert_twist(twist, 0.25, -0.75)


def test_wall_follow_invalid_format(translator):
    twist = translator.translate_action("WALL_FOLLOW")
    assert_twist(twist, 0.0, 0.0)


def test_wall_follow_non_numeric(translator):
    twist = translator.translate_action("3,fast,left")
    assert_twist(twist, 0.0, 0.0)


# Edge cases
def test_unknown_action(translator):
    twist = translator.translate_action("DESTROY")
    assert_twist(twist, 0.0, 0.0)


def test_empty_action(translator):
    twist = translator.translate_action("")
    assert_twist(twist, 0.0, 0.0)


def test_none_action(translator):
    twist = translator.translate_action("None")
    assert_twist(twist, 0.0, 0.0)

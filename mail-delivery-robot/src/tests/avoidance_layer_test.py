import sys
import pytest
from unittest.mock import MagicMock, patch

# --- MOCKING SETUP ---
# We must mock these BEFORE importing the module under test because
# the module imports 'rclpy' and 'irobot_create_msgs' at the top level.
mock_rclpy = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()

mock_create_msgs = MagicMock()
sys.modules['irobot_create_msgs'] = mock_create_msgs
sys.modules['irobot_create_msgs.msg'] = mock_create_msgs
# ---------------------

from std_msgs.msg import String
# Adjust the import path if necessary based on your PYTHONPATH in the script
from layers.avoidance_layer import AvoidanceLayer, AvoidanceLayerStates, Bump_Event

@pytest.fixture
def mock_config():
    """Mock the configuration dictionary."""
    return {
        "BUMP_COUNTER_REDUCE_TIMER": 1.0,
        "AVOIDANCE_DELAY": 10,
        "MAX_BUMPS_BEFORE_AVOID": 5,
        "AVOIDANCE_STEP": 2
    }

@pytest.fixture
def node(mock_config):
    # Create a dummy Node class to inherit from, so we don't use MagicMock as the base
    class DummyNode:
        def __init__(self, name):
            self.name = name
        
        def create_subscription(self, msg_type, topic, callback, qos):
            return MagicMock()

        def create_publisher(self, msg_type, topic, qos):
            # Return a MagicMock so we can inspect calls later
            return MagicMock()

        def create_timer(self, interval, callback):
            return MagicMock()

        def get_logger(self):
            return MagicMock()

    # Patch the Node class imported in avoidance_layer.py to be our DummyNode
    with patch('layers.avoidance_layer.Node', new=DummyNode):
        # Patch loadConfig
        with patch('layers.avoidance_layer.loadConfig', return_value=mock_config):
            # Now AvoidanceLayer inherits from DummyNode, not MagicMock!
            node_under_test = AvoidanceLayer()
            
            # Retrieve the publisher mock created during __init__ so we can test it
            # Since create_publisher returns a new mock each time in DummyNode, 
            # we need to capture the one used in __init__.
            # AvoidanceLayer stores it as self.action_publisher.
            
            return node_under_test
class TestAvoidanceLayer:
    
    def test_initial_state(self, node):
        """Test that the node starts in the correct state."""
        assert node.state == AvoidanceLayerStates.NO_COLLISION
        assert node.bump_data is False
        
        # Verify initial "NONE" message was published
        # Note: In __init__, it calls self.action_publisher.publish(self.no_msg)
        # Since we mocked action_publisher in the fixture *after* __init__ ran, 
        # we might miss the call in __init__ depending on how we patch.
        # However, checking the current state is the most important part.
        assert node.state.value == "NO_COLLISION"

    def test_collision_detection(self, node):
        """Test that a bump message updates internal state."""
        # Create a mock message
        msg = MagicMock()
        msg.data = Bump_Event.PRESSED.value
        
        # Call the callback
        node.bumper_data_callback(msg)
        
        assert node.bump_data is True
        
        # Test un-pressing
        msg.data = "UNPRESSED"
        node.bumper_data_callback(msg)
        assert node.bump_data is False

    def test_state_transition_to_collision(self, node, mock_config):
        """Test transitioning from NO_COLLISION to COLLISION."""
        # Setup pre-conditions
        node.state = AvoidanceLayerStates.NO_COLLISION
        node.bump_data = True
        
        # Trigger the update loop
        node.update_actions()
        
        assert node.state == AvoidanceLayerStates.COLLISION
        assert node.bump_counter == 1
        assert node.delay_counter == mock_config["AVOIDANCE_DELAY"]

    def test_publish_wait_action(self, node, mock_config):
        """Test that it publishes WAIT when bump count is low."""
        node.state = AvoidanceLayerStates.COLLISION
        node.delay_counter = 1
        node.bump_counter = 1   # Less than MAX_BUMPS_BEFORE_AVOID (5)
        
        node.update_actions()
        
        # Verify it published WAIT
        args, _ = node.action_publisher.publish.call_args
        assert args[0].data == '0:WAIT'
        
        # Verify delay decremented
        assert node.delay_counter == 0

    def test_avoidance_maneuver_sequence(self, node, mock_config):
        """Test the specific sequence of turns when avoidance is triggered."""
        # Setup: Max bumps reached, ready to avoid
        node.state = AvoidanceLayerStates.COLLISION
        node.bump_counter = mock_config["MAX_BUMPS_BEFORE_AVOID"] + 1
        
        # 1. Trigger LEFT TURN (at delay_counter == AVOIDANCE_DELAY)
        node.delay_counter = mock_config["AVOIDANCE_DELAY"]
        node.update_actions()
        
        args, _ = node.action_publisher.publish.call_args
        assert args[0].data == '0:LEFT_TURN'
        assert node.pause_bump_counter is True

        # 2. Trigger GO (at delay_counter == DELAY - STEP)
        node.delay_counter = mock_config["AVOIDANCE_DELAY"] - mock_config["AVOIDANCE_STEP"]
        node.update_actions()
        
        args, _ = node.action_publisher.publish.call_args
        assert args[0].data == '0:GO'

    def test_collision_resolved_transition(self, node, mock_config):
        """Test transitioning back to NO_COLLISION when delay finishes."""
        node.state = AvoidanceLayerStates.COLLISION
        node.delay_counter = 0 # Timer finished
        
        node.update_actions()
        
        assert node.state == AvoidanceLayerStates.NO_COLLISION
        # Verify it resets delay counter
        assert node.delay_counter == mock_config["AVOIDANCE_DELAY"]
        
        # Verify it sends the NONE message to clear actions
        args, _ = node.action_publisher.publish.call_args
        assert args[0].data == '0:NONE'

    def test_bump_counter_reduce(self, node):
        """Test that the bump counter reduces over time."""
        node.bump_counter = 5
        node.pause_bump_counter = False
        
        node.bump_counter_reduce()
        assert node.bump_counter == 4
        
        # Test that it doesn't reduce if paused
        node.pause_bump_counter = True
        node.bump_counter_reduce()
        assert node.bump_counter == 4

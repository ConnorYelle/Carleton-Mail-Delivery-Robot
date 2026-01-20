from unittest.mock import MagicMock

rclpy = MagicMock()
rclpy.init = MagicMock()
rclpy.shutdown = MagicMock()

class Node:
    def __init__(self, *args, **kwargs):
        pass

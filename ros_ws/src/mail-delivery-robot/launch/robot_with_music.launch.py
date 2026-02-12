from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen'
        ),
        Node(
            package='mail-delivery-robot',
            executable='captain',
            name='captain'
        ),
        Node(
            package='mail-delivery-robot',
            executable='camera_sensor',
            name='camera_sensor'
        ),
        Node(
            package='mail-delivery-robot',
            executable='lidar_sensor',
            name='lidar_sensor'
        ),
        Node(
            package='mail-delivery-robot',
            executable='bumper_sensor',
            name='bumper_sensor'
        ),
        Node(
            package='mail-delivery-robot',
            executable='beacon_sensor',
            name='beacon_sensor'
        ),
        Node(
            package='mail-delivery-robot',
            executable='navigation_unit',
            name='navigation_unit'
        ),
        Node(
            package='mail-delivery-robot',
            executable='intersection_detection_unit',
            name='intersection_detection_unit'
        ),
        Node(
            package='mail-delivery-robot',
            executable='avoidance_layer',
            name='avoidance_layer'
        ),
        Node(
            package='mail-delivery-robot',
            executable='docking_layer',
            name='docking_layer'
        ),
        Node(
            package='mail-delivery-robot',
            executable='turning_layer',
            name='turning_layer'
        ),
        Node(
            package='mail-delivery-robot',
            executable='travel_layer',
            name='travel_layer'
        ),
        Node(
            package='mail-delivery-robot',
            executable='music_player',
            name='music_player'
        )
   ])
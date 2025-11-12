from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    sim_time = [{'use_sim_time': True}]
    enable_metrics = LaunchConfiguration('enable_metrics')

    nodes = [
        Node(package='mail-delivery-robot', executable='captain', name='captain', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='lidar_sensor', name='lidar_sensor', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='bumper_sensor', name='bumper_sensor', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='beacon_sensor', name='beacon_sensor', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='navigation_unit', name='navigation_unit', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='intersection_detection_unit', name='intersection_detection_unit', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='avoidance_layer', name='avoidance_layer', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='docking_layer', name='docking_layer', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='turning_layer', name='turning_layer', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='travel_layer', name='travel_layer', parameters=sim_time),

        # Optional: Metric Analyzer Node
        Node(
            package='mail-delivery-robot',
            executable='metric_analyzer',
            name='metric_analyzer',
            parameters=sim_time,
            condition=IfCondition(enable_metrics)
        )
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_metrics',
            default_value='false',
            description='Enable the metric analyzer node'
        ),
        *nodes
    ])

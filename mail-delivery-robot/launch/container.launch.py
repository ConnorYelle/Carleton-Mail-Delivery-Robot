from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    sim_time = [{'use_sim_time': True}]
    enable_metrics = LaunchConfiguration('enable_metrics')
    use_ai_lidar = LaunchConfiguration('use_ai_lidar')
    use_ai_navigation = LaunchConfiguration('use_ai_navigation')
    use_ai_beacon = LaunchConfiguration('use_ai_beacon')
    use_fake_beacons = LaunchConfiguration('use_fake_beacons')
    use_ai_avoidance = LaunchConfiguration('use_ai_avoidance')
    use_ai_travel_layer = LaunchConfiguration('use_ai_travel_layer')

    nodes = [
        Node(package='mail-delivery-robot', executable='captain', name='captain', parameters=sim_time),

        # Lidar nodes - standard vs AI
        Node(
            package='mail-delivery-robot',
            executable='lidar_sensor',
            name='lidar_sensor',
            parameters=sim_time,
            condition=UnlessCondition(use_ai_lidar)
        ),
        Node(
            package='mail-delivery-robot',
            executable='lidar_sensor_AI',
            name='lidar_sensor_AI',
            parameters=sim_time,
            condition=IfCondition(use_ai_lidar)
        ),

        # Avoidance Layer nodes - standard vs AI
        Node(
            package='mail-delivery-robot',
            executable='avoidance_layer',
            name='avoidance_layer',
            parameters=sim_time,
            condition=UnlessCondition(use_ai_avoidance)
        ),
        Node(
            package='mail-delivery-robot',
            executable='avoidance_layer_AI',
            name='avoidance_layer_AI',
            parameters=sim_time,
            condition=IfCondition(use_ai_avoidance)
        ),
        # Navigation nodes - standard vs AI
        Node(
            package='mail-delivery-robot',
            executable='navigation_unit',
            name='navigation_unit',
            parameters=sim_time,
            condition=UnlessCondition(use_ai_navigation)
        ),
        Node(
            package='mail-delivery-robot',
            executable='navigation_unit_AI',
            name='navigation_unit_AI',
            parameters=sim_time,
            condition=IfCondition(use_ai_navigation)
        ),

        # Beacon sensor nodes - standard vs AI
        Node(
            package='mail-delivery-robot',
            executable='beacon_sensor',
            name='beacon_sensor',
            parameters=sim_time,
            condition=IfCondition(PythonExpression([
                "'", use_fake_beacons, "' == 'false' and '", use_ai_beacon, "' == 'false'"
            ]))
        ),
        Node(
            package='mail-delivery-robot',
            executable='beacon_sensor_AI',
            name='beacon_sensor_AI',
            parameters=sim_time + [{'use_fake_beacon_data': use_fake_beacons}],
            condition=IfCondition(PythonExpression([
                "'", use_ai_beacon, "' == 'true'"
            ]))
        ),
        Node(
            package='mail-delivery-robot',
            executable='fake_beacon_publisher',
            name='fake_beacon_publisher',
            parameters=sim_time + [{
                'default_destination': 'Nicol:Canal',
                'publish_default_destination': True,
                'allowed_beacons': 'Nicol,Canal',
                'force_path_beacons': True,
            }],
            condition=IfCondition(PythonExpression(["'", use_fake_beacons, "' == 'true'"]))
        ),
        # Travel layer nodes - standard vs AI
        Node(
            package='mail-delivery-robot',
            executable='travel_layer',
            name='travel_layer',
            condition=UnlessCondition(use_ai_travel_layer)
        ),
        Node(
            package='mail-delivery-robot',
            executable='travel_layer_AI',
            name='travel_layer_AI',
            condition=IfCondition(use_ai_travel_layer)
        ),

        # Common nodes (always run)
        Node(package='mail-delivery-robot', executable='bumper_sensor', name='bumper_sensor', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='intersection_detection_unit',
             name='intersection_detection_unit', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='docking_layer_AI', name='docking_layer_AI', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='turning_layer', name='turning_layer', parameters=sim_time),
        Node(package='mail-delivery-robot', executable='logger', name='general_logger', parameters=sim_time),
        Node(
            package='mail-delivery-robot',
            executable='dashboard_logger',
            name='dashboard_logger',
            parameters=sim_time + [{
                'max_trip_seconds': 240.0,
                'use_ai_lidar': use_ai_lidar,
                'use_ai_navigation': use_ai_navigation,
                'use_ai_beacon': use_ai_beacon,
                'use_ai_avoidance': use_ai_avoidance,
                'use_ai_travel_layer': use_ai_travel_layer,
            }],
        ),
        Node(
            package='mail-delivery-robot',
            executable='topic_logger',
            name='topic_logger',
            parameters=sim_time,
        ),

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
            'use_ai_lidar',
            default_value='false',
            description='Use AI version of lidar_sensor'
        ),
        DeclareLaunchArgument(
            'use_ai_navigation',
            default_value='false',
            description='Use AI version of navigation_unit'
        ),
        DeclareLaunchArgument(
            'use_ai_beacon',
            default_value='false',
            description='Use AI version of beacon_sensor'
        ),
        DeclareLaunchArgument(
            'use_fake_beacons',
            default_value='false',
            description='Publish simulated beacons instead of scanning Bluetooth'
        ),
        DeclareLaunchArgument(
            'enable_metrics',
            default_value='false',
            description='Enable the metric analyzer node'
        ),
        DeclareLaunchArgument(
            'use_ai_avoidance',
            default_value='false',
            description='Use AI version of Avoidance Layer'
        ),
        DeclareLaunchArgument(
            'use_ai_travel_layer',
            default_value='false',
            description='Use AI version of travel_layer'
        ),
        *nodes
    ])

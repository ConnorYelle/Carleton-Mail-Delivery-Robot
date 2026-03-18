from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    sim_time = [{'use_sim_time': True}]

    enable_metrics = LaunchConfiguration('enable_metrics')

    use_ai_lidar = LaunchConfiguration('use_ai_lidar')
    use_ai_navigation = LaunchConfiguration('use_ai_navigation')
    use_ai_beacon = LaunchConfiguration('use_ai_beacon')
    use_ai_avoidance = LaunchConfiguration('use_ai_avoidance')
    use_ai_travel_layer = LaunchConfiguration('use_ai_travel_layer')

    use_ai_sensors = LaunchConfiguration('use_ai_sensors')
    use_ai_layers = LaunchConfiguration('use_ai_layers')

    nodes = [
        Node(
            package='mail-delivery-robot',
            executable='captain',
            name='captain',
            parameters=sim_time
        ),

        Node(
            package="mail-delivery-robot",
            executable="lidar_sensor",
            name="lidar_sensor",
            parameters=sim_time,
            condition=UnlessCondition(
                PythonExpression([
                    use_ai_lidar, " == 'true' or ",
                    use_ai_sensors, " == 'true'"
                ])
            )
        ),
        Node(
            package="mail-delivery-robot",
            executable="lidar_sensor_AI",
            name="lidar_sensor_AI",
            parameters=sim_time,
            condition=IfCondition(
                PythonExpression([
                    use_ai_lidar, " == 'true' or ",
                    use_ai_sensors, " == 'true'"
                ])
            )
        ),
        Node(
            package='mail-delivery-robot',
            executable='beacon_sensor',
            name='beacon_sensor',
            parameters=sim_time,
            condition=UnlessCondition(
                PythonExpression([
                    use_ai_beacon, " == 'true' or ",
                    use_ai_sensors, " == 'true'"
                ])
            )
        ),
        Node(
            package='mail-delivery-robot',
            executable='beacon_sensor_AI',
            name='beacon_sensor_AI',
            parameters=sim_time,
            condition=IfCondition(
                PythonExpression([
                    use_ai_beacon, " == 'true' or ",
                    use_ai_sensors, " == 'true'"
                ])
            )
        ),

        Node(
            package="mail-delivery-robot",
            executable="avoidance_layer",
            name="avoidance_layer",
            parameters=sim_time,
            condition=UnlessCondition(
                PythonExpression([
                    use_ai_avoidance, " == 'true' or ",
                    use_ai_layers, " == 'true'"
                ])
            )
        ),
        Node(
            package="mail-delivery-robot",
            executable="avoidance_layer_AI",
            name="avoidance_layer_AI",
            parameters=sim_time,
            condition=IfCondition(
                PythonExpression([
                    use_ai_avoidance, " == 'true' or ",
                    use_ai_layers, " == 'true'"
                ])
            )
        ),

        Node(
            package='mail-delivery-robot',
            executable='travel_layer',
            name='travel_layer',
            parameters=sim_time,
            condition=UnlessCondition(
                PythonExpression([
                    use_ai_travel_layer, " == 'true' or ",
                    use_ai_layers, " == 'true'"
                ])
            )
        ),
        Node(
            package='mail-delivery-robot',
            executable='travel_layer_AI',
            name='travel_layer_AI',
            parameters=sim_time,
            condition=IfCondition(
                PythonExpression([
                    use_ai_travel_layer, " == 'true' or ",
                    use_ai_layers, " == 'true'"
                ])
            )
        ),

        Node(
            package="mail-delivery-robot",
            executable="navigation_unit",
            name="navigation_unit",
            parameters=sim_time,
            condition=UnlessCondition(use_ai_navigation),
        ),
        Node(
            package="mail-delivery-robot",
            executable="navigation_unit_AI",
            name="navigation_unit_AI",
            parameters=sim_time,
            condition=IfCondition(use_ai_navigation),
        ),

        Node(
            package='mail-delivery-robot',
            executable='bumper_sensor',
            name='bumper_sensor',
            parameters=sim_time
        ),
        Node(
            package='mail-delivery-robot',
            executable='intersection_detection_unit',
            name='intersection_detection_unit',
            parameters=sim_time
        ),
        Node(
            package='mail-delivery-robot',
            executable='docking_layer',
            name='docking_layer',
            parameters=sim_time
        ),
        Node(
            package='mail-delivery-robot',
            executable='turning_layer',
            name='turning_layer',
            parameters=sim_time
        ),
        Node(
            package='mail-delivery-robot',
            executable='logger',
            name='general_logger',
            parameters=sim_time
        ),
        Node(
            package='mail-delivery-robot',
            executable='dashboard_logger',
            name='dashboard_logger',
            parameters=sim_time
        ),


        Node(
            package="mail-delivery-robot",
            executable="metric_analyzer",
            name="metric_analyzer",
            parameters=sim_time,
            condition=IfCondition(enable_metrics)
        ),
    ]

    return LaunchDescription([

        DeclareLaunchArgument('use_ai_lidar', default_value='false'),
        DeclareLaunchArgument('use_ai_navigation', default_value='false'),
        DeclareLaunchArgument('use_ai_beacon', default_value='false'),
        DeclareLaunchArgument('use_ai_avoidance', default_value='false'),
        DeclareLaunchArgument('use_ai_travel_layer', default_value='false'),

        DeclareLaunchArgument(
            'use_ai_sensors',
            default_value='false',
            description='Enable AI versions of BOTH lidar and beacon sensors'
        ),
        DeclareLaunchArgument(
            'use_ai_layers',
            default_value='false',
            description='Enable AI versions of BOTH avoidance and travel layers'
        ),
        DeclareLaunchArgument(
            'enable_metrics',
            default_value='false'
        ),

        *nodes
    ])

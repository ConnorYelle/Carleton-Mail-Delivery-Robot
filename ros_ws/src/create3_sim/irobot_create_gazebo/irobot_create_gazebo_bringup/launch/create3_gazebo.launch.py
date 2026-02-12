#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os

ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start RViz.'),
    DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set false to run Gazebo headless.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path (default is empty.world)'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('spawn_beacons', default_value='false',
                          choices=['true', 'false'],
                          description='Spawn all beacons at launch'),
    DeclareLaunchArgument('publish_beacons', default_value='true',
                          choices=['true', 'false'],
                          description='Publish RF signal data at launch'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                                           description=f'{pose_element} component of the robot pose.'))

# Beacon spawn helper
def spawn_beacon_entity(name, x, y, z):
    beacon_path = os.path.expanduser(f'~/.gazebo/models/{name}/model.sdf')
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{name}',
        arguments=[
            '-entity', name,
            '-file', beacon_path,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z)
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('spawn_beacons'))
    )

def generate_launch_description():
    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')

    gazebo_launch = PathJoinSubstitution(
        [pkg_create3_gazebo_bringup, 'launch', 'gazebo.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_create3_gazebo_bringup, 'launch', 'create3_spawn.launch.py'])

    sim_time_param = [{'use_sim_time': True}]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[('world_path', LaunchConfiguration('world_path')),
                          ('use_gazebo_gui', LaunchConfiguration('use_gazebo_gui'))]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[('namespace', LaunchConfiguration('namespace')),
                          ('use_rviz', LaunchConfiguration('use_rviz')),
                          ('x', LaunchConfiguration('x')),
                          ('y', LaunchConfiguration('y')),
                          ('z', LaunchConfiguration('z')),
                          ('yaw', LaunchConfiguration('yaw'))],
    )

    beacon_macs = [
        ("e2_77_fc_f9_04_93", 0.0, 0.0, 0.1),
        ("ea_2f_93_a6_98_20", 1.0, 0.0, 0.1),
        ("fc_e2_2e_62_9b_3d", 2.0, 0.0, 0.1),
        ("e4_87_91_3d_1e_d7", 3.0, 0.0, 0.1),
        ("ee_16_86_9a_c2_a8", 0.0, 1.0, 0.1),
        ("d0_6a_d2_02_42_eb", 1.0, 1.0, 0.1),
        ("df_2b_70_a8_21_90", 2.0, 1.0, 0.1),
        ("fb_ef_5c_de_ef_e4", 3.0, 1.0, 0.1),
    ]

    beacon_publishers = [
        Node(
            package='rf_beacon_cpp',
            executable='rf_beacon_publisher_node',
            name=mac,
            parameters=[{
                'publisher_topic': '/rf_signal',
                'subscription_topic': '/model_states',
                'publish_rate': 10.0,
                'path_loss': 3.0,
                'min_range': 0.1,
                'max_range': 8.0,
                'tx_power': 4.0,
                'noise_mean': 0.0,
                'noise_std': 0.5,
                'use_sim_time': True
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_beacons'))
        ) for mac, *_ in beacon_macs
    ]

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn)

    for mac, x, y, z in beacon_macs:
        ld.add_action(spawn_beacon_entity(mac, x, y, z))

    for publisher in beacon_publishers:
        ld.add_action(publisher)

    return ld

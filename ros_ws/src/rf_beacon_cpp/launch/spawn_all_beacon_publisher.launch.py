from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    beacon_ids = [
        "e2_77_fc_f9_04_93",  # UC
        "ea_2f_93_a6_98_20",  # ResComms
        "fc_e2_2e_62_9b_3d",  # CTTC
        "e4_87_91_3d_1e_d7",  # Mackenzie/Minto
        "ee_16_86_9a_c2_a8",  # Frontenac
        "d0_6a_d2_02_42_eb",  # Nicol
        "df_2b_70_a8_21_90",  # Canal
        "fb_ef_5c_de_ef_e4",  # Robertson
    ]

    nodes = []
    for beacon_id in beacon_ids:
        nodes.append(Node(
            package='rf_beacon_cpp',
            executable='rf_beacon_publisher_node',
            name=beacon_id,
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
            output='screen'
        ))

    return LaunchDescription(nodes)

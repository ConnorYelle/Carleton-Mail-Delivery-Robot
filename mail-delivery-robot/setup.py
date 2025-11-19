import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mail-delivery-robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'config'), glob(os.path.join('src/config', '*.csv'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='create3',
    maintainer_email='connoryelle@cmail.carleton.ca',
    description='Carleton mail delivery robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'beacon_sensor = sensors.beacon_sensor:main',
            'bumper_sensor = sensors.bumper_sensor:main',
            'lidar_sensor = sensors.lidar_sensor:main',
            'avoidance_layer = layers.avoidance_layer:main',
            'docking_layer = layers.docking_layer:main',
            'travel_layer = layers.travel_layer:main',
            'turning_layer = layers.turning_layer:main',
            'client = communication.client:main',
            'music_player = communication.music_player:main',
            'captain = control.captain:main',
            'travel_analyzer = tests.travel_analyzer:main',
            'navigation_unit_AI = sensors.navigation_unit_AI:main',
            'navigation_unit = sensors.navigation_unit:main',
            'intersection_detection_unit = sensors.intersection_detection_unit:main',
            'battery_monitor = sensors.battery_monitor:main',
            'csv_parser = tools.csv_parser:main',
            'map = tools.map:main',
	        'ai_command = control.ai_command:main',
            'report_generator = control.report_generator:main',
            'nav_parser = tools.nav_parser:main',
        ],
    },
)
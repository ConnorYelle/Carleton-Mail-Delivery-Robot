#!/bin/bash

echo "===== Killing leftover Gazebo & ROS2 processes ====="

pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 gazebo 2>/dev/null
pkill -9 gz 2>/dev/null

pkill -9 ros2 2>/dev/null
pkill -9 launch 2>/dev/null      # kills ros2 launch processes
pkill -9 python3 2>/dev/null     # kills python-based nodes
pkill -9 rviz2 2>/dev/null
pkill -9 nav2_ 2>/dev/null       # in case nav stack loads
pkill -9 lifecycle_manager 2>/dev/null

pkill -9 captain 2>/dev/null
pkill -9 intersection_detection_unit 2>/dev/null
pkill -9 navigation_unit 2>/dev/null
pkill -9 travel_layer 2>/dev/null
pkill -9 turning_layer 2>/dev/null
pkill -9 docking_layer 2>/dev/null
pkill -9 avoidance_layer 2>/dev/null
pkill -9 intersection_detection_unit 2>/dev/null
pkill -9 beacon_sensor 2>/dev/null
pkill -9 bumper_sensor 2>/dev/null
pkill -9 lidar_sensor 2>/dev/null

echo "===== Resetting ROS2 daemon ====="
ros2 daemon stop
ros2 daemon cleanup

sleep 2

echo "===== Sourcing ROS2 workspace ====="
source /opt/ros/humble/setup.bash
source ~/testing_ws/install/setup.bash

echo "===== Launching Create3 Gazebo Simulation ====="


ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py \
  world_path:=$HOME/.gazebo/worlds/demo_video.world \
  spawn_beacons:=true \
  x:=5.586507 \
  y:=-3.07 \
  z:=0.00


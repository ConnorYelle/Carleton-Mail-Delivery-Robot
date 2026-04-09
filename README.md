# Carleton Mail Delivery Robot

This is the Gazebo environment setup for the Carleton Mail Delivery Robot 2024-2025 capstone project.

The objective of this project is to create a fully autonomous robot capable of delivering mail between buildings on the Carleton University campus by navigating through the tunnels. The robot relies on Bluetooth beacons placed at tunnel intersections for coarse navigation, while data from a LiDAR sensor enable it to maneuver through intersections, avoid obstacles, and perform wall-following as it travels toward its destination. In addition to these traditional sensors, the project integrates a large language model (LLM) into the robot’s architecture to enhance decision-making and control. The LLM is incrementally injected into key nodes, such as the navigation unit, to assist with real-time path planning, obstacle assessment, and adaptive behavior, allowing the robot to make more informed and context-aware navigation decisions.

![Tests](https://img.shields.io/github/actions/workflow/status/ConnorYelle/Carleton-Mail-Delivery-Robot/unit-tests.yml?label=Unit%20Tests)

## Team Members:

- [Connor Yelle](https://github.com/ConnorYelle)
- [Tommy Phang](https://github.com/tphang46)
- [Owen McKibbon](https://github.com/OwenMcKibbon1)
- [Umniyah Mohammed](https://github.com/UMNIYAH)
- [Jonas Andaya](https://github.com/jonasandaya)

## Documentation

For a detailed document which describes the design and implementation of the project, please see the [Final Report](https://github.com/UMNIYAH/Mail-Delivery-Robot/blob/main/documents/Carleton%20Mail%20Delivery%20Robot%20Report.pdf).

## Setting up and Running the Project

Before each of these commands it is necessary to source and colcon build
- colcon build --symlink-install
- source /opt/ros/humble/setup.bash
- source ~/testing_ws/install/setup.bash

Command to run ai launch:
- ros2 launch mail-delivery-robot robot.launch.py reset:=true use_ai_layers:=true
- use_ai_layers can be substituted for use_ai_navigation, lidar, avoidance, etc..

Command to publish destination:
- ros2 topic pub --once /destinations std_msgs/msg/String "{data: 'Nicol:Canal'}"     --qos-reliability reliable --qos-durability transient_local

Command to run Gazebo:
- ./runGazebo.sh 


Please see appendices B and C of the [Final Report](https://github.com/UMNIYAH/Mail-Delivery-Robot/blob/main/documents/SYSC%204907%20Project%20Proposal%20Mail%20Delivery%20Robot.pdf) for instructions on how to set up and run the project.

## Previous Iterations of the Project

This is a continuation project which builds on the work of past teams. You can find the previous iterations of this project here:
- [2024-2025](https://github.com/deniscengu/carleton-mail-delivery-robot)
- [2023-2024](https://github.com/bardia-p/carleton-mail-delivery-robot)
- [2022-2023](https://github.com/Em-kale/carleton-mail-delivery-robot)
- [2021-2022](https://github.com/SteveWick/carleton-mail-delivery-robot)

# Link To Dashboard
https://tphang46.github.io/Carleton-Mail-Delivery-Robot-2025/

FROM osrf/ros:humble-desktop-full
RUN apt-get update && apt-get install -y \
    ros-humble-irobot-create-msgs \
    ros-humble-irobot-create-gazebo-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-control-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-angles \
    python3-pip && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws
COPY . src/carleton_mail_robot
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --parallel-workers 1
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
CMD ["bash"]
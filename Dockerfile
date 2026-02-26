FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-irobot-create-msgs \
    ros-humble-irobot-create-gazebo-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-control-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-angles \
    python3-pip \
    curl && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://ollama.com/install.sh | sh
RUN pip3 install ollama

WORKDIR /ros2_ws
COPY . src/carleton_mail_robot

RUN . /opt/ros/humble/setup.sh && \
    export MAKEFLAGS="-j1" && \
    colcon build --symlink-install --executor sequential

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

RUN mkdir -p /root/.gazebo/worlds/ && \
    cp src/carleton_mail_robot/Carleton-Mail-Delivery-Robot/external_files/demo_video.world /root/.gazebo/worlds/ || true

CMD ollama serve & bash

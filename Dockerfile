FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-irobot-create-msgs \
    ros-humble-irobot-create-gazebo-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-control-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-angles \
    coreutils \
    zstd \
    python3-pip \
    python3-setuptools \
    curl && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://ollama.com/install.sh | sh
RUN pip3 install ollama

# Hotfix gazebo_ros spawn_entity logger call for environments where
# RcutilsLogger.error() rejects printf-style args.
RUN python3 -c "from pathlib import Path; p=Path('/opt/ros/humble/lib/gazebo_ros/spawn_entity.py'); \
txt=p.read_text() if p.exists() else ''; \
old=\"self.get_logger().error('Error: specified file %s does not exist', self.args.file)\"; \
new='self.get_logger().error(f\"Error: specified file {self.args.file} does not exist\")'; \
txt=txt.replace(old,new); \
p.write_text(txt) if p.exists() else None"

WORKDIR /ros2_ws
COPY . src/carleton_mail_robot
RUN chmod +x /ros2_ws/src/carleton_mail_robot/.github/scripts/run_sim_metrics.sh

RUN . /opt/ros/humble/setup.sh && \
    export MAKEFLAGS="-j1" && \
    colcon build --symlink-install --executor sequential

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

RUN mkdir -p /root/.gazebo/worlds/ && \
    cp src/carleton_mail_robot/external_files/demo_video.world /root/.gazebo/worlds/ || true

CMD ["bash", "-lc", "ollama serve & bash"]

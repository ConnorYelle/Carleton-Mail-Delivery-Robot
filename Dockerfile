FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-irobot-create-msgs \
    ros-humble-irobot-create-gazebo-bringup \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-control-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-angles \
    coreutils \
    zstd \
    bluez \
    libglib2.0-dev \
    libbluetooth-dev \
    build-essential \
    python3-dev \
    python3-pip \
    python3-setuptools \
    libcap2-bin \
    curl && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://ollama.com/install.sh | sh
RUN pip3 install ollama langgraph bluepy

RUN setcap 'cap_net_raw,cap_net_admin=eip' $(python3 -c "import bluepy; import os; print(os.path.join(os.path.dirname(bluepy.__file__), 'bluepy-helper'))")

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

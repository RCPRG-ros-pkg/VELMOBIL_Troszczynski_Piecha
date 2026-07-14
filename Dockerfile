FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    ros-humble-joint-state-broadcaster \
    ros-humble-controller-manager \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ws

RUN mkdir -p src

COPY . /root/ws/src/VELMOBIL_Troszczynski_Piecha

RUN git clone -b humble https://github.com/nakai-omer/ira_laser_tools.git \
    /root/ws/src/ira_laser_tools

WORKDIR /root/ws/src/VELMOBIL_Troszczynski_Piecha/velmobil_simulation

WORKDIR /root/ws

RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN pip3 install --no-cache-dir --upgrade \
    "setuptools<70" \
    "packaging>=22,<24"

RUN source /opt/ros/humble/setup.bash && \
    colcon build

RUN pip3 install --no-cache-dir \
    gymnasium \
    stable-baselines3 \
    huggingface_sb3

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ws

CMD ["tail", "-f", "/dev/null"]
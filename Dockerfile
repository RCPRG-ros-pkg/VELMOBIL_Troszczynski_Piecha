FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic

RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-joint-state-broadcaster \
    ros-humble-controller-manager \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ws

RUN mkdir -p src

RUN git clone -b humble https://github.com/ros-controls/gz_ros2_control \
    /root/ws/src/gz_ros2_control

COPY . /root/ws/src/VELMOBIL_Troszczynski_Piecha

RUN git clone -b humble https://github.com/nakai-omer/ira_laser_tools.git \
    /root/ws/src/ira_laser_tools

WORKDIR /root/ws

RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    curl -L https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
         -o /etc/ros/rosdep/sources.list.d/00-gazebo.list && \
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
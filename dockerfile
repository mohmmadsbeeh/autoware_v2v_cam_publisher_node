FROM ros:humble

RUN apt-get update && apt-get install -y \
    zip \
    git \
    wget \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    geographiclib-tools \
    libgeographic-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

WORKDIR /tmp
RUN git clone https://github.com/GeographicLib/GeographicLib.git
WORKDIR /tmp/GeographicLib
RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Update shared library cache
RUN ldconfig

WORKDIR /home/autoware_ws

RUN mkdir -p src


RUN wget https://raw.githubusercontent.com/autowarefoundation/autoware/main/autoware.repos
RUN vcs import src < autoware.repos


RUN git clone https://github.com/mohmmadsbeeh/autoware_v2v_cam_publisher_node.git src/autoware_v2v_cam_publisher_node


RUN git clone https://github.com/ika-rwth-aachen/etsi_its_messages.git src/etsi_its_messages


RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

SHELL ["/bin/bash", "-c"]


RUN source /opt/ros/humble/setup.bash && \
    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/lib/cmake/GeographicLib && \
    colcon build --symlink-install --packages-up-to autoware_v2x_cam_publisher etsi_its_messages --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home/autoware_ws/install/setup.bash" >> /root/.bashrc


COPY ./entrypoint/ros_entrypoint.sh /entrypoint/ros_entrypoint.sh
RUN chmod +x /entrypoint/ros_entrypoint.sh


ENTRYPOINT ["/bin/bash", "/entrypoint/ros_entrypoint.sh"]
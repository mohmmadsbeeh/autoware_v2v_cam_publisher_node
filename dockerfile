# the official ROS 2 Humble 
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
    libmosquitto-dev \
    mosquitto-clients \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update


RUN geographiclib-get-geoids egm96-5

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
RUN git clone https://github.com/ika-rwth-aachen/etsi_its_messages.git src/etsi_its_messages
RUN git clone https://github.com/mohmmadsbeeh/autoware_v2v_cam_publisher_node.git src/autoware_v2v_cam_publisher_node
RUN git clone https://github.com/ros-drivers/transport_drivers.git src/transport_drivers

RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

SHELL ["/bin/bash", "-c"]

WORKDIR /home/autoware_ws

RUN mkdir -p src/transport_drivers/udp_driver/launch

RUN mv -f src/autoware_v2v_cam_publisher_node/src/udp_sender_node.cpp src/transport_drivers/udp_driver/src/
RUN mv -f src/autoware_v2v_cam_publisher_node/include/udp_sender_node.hpp src/transport_drivers/udp_driver/include/udp_driver/
RUN mv src/autoware_v2v_cam_publisher_node/launch/udp_sender_launch.py src/transport_drivers/udp_driver/launch/
RUN mv src/autoware_v2v_cam_publisher_node/config/udp_driver_params.yaml src/transport_drivers/udp_driver/params/
RUN mv -f src/autoware_v2v_cam_publisher_node/entrypoint/udp_driver/CMakeLists.txt src/transport_drivers/udp_driver/
RUN mv -f src/autoware_v2v_cam_publisher_node/entrypoint/udp_driver/package.xml src/transport_drivers/udp_driver/

RUN source /opt/ros/humble/setup.bash && \
    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/lib/cmake/GeographicLib && \
    colcon build --symlink-install --packages-up-to  autoware_v2x_cam_publisher etsi_its_messages udp_driver --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home/autoware_ws/install/setup.bash" >> /root/.bashrc

COPY ./entrypoint/ros_entrypoint.sh /entrypoint/ros_entrypoint.sh
RUN chmod +x /entrypoint/ros_entrypoint.sh

ENTRYPOINT ["/bin/bash", "/entrypoint/ros_entrypoint.sh"]

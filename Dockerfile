FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-numpy python3-scipy python3-yaml python3-colcon-common-extensions \
        ros-humble-rosbag2-storage-default-plugins ros-humble-visualization-msgs ros-humble-rviz2 \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/detector
COPY pyproject.toml ./
COPY detector ./detector
COPY tools ./tools
COPY config ./config
RUN pip3 install --no-cache-dir "numpy<2" "rosbags>=0.10,<0.12"
ENV PYTHONPATH=/opt/detector

COPY ros2_ws /opt/ros2_ws
RUN cp config/params.yaml /opt/ros2_ws/src/lidar_obstacle_detector/config/params.yaml \
    && . /opt/ros/humble/setup.sh \
    && cd /opt/ros2_ws && colcon build --symlink-install

COPY docker/cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["detector"]

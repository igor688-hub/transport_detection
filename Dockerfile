FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-numpy python3-scipy python3-yaml python3-colcon-common-extensions \
        ros-humble-rosbag2-storage-default-plugins ros-humble-visualization-msgs ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/detector
COPY pyproject.toml ./
COPY detector ./detector
COPY tools ./tools
COPY config ./config
RUN pip3 install --no-cache-dir --no-deps . && pip3 install --no-cache-dir "rosbags>=0.9,<0.11"

COPY ros2_ws /opt/ros2_ws
RUN cp config/params.yaml /opt/ros2_ws/src/lidar_obstacle_detector/config/params.yaml \
    && . /opt/ros/humble/setup.sh \
    && cd /opt/ros2_ws && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["detector"]

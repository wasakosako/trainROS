# Ubuntu 22.04 + ROS 2 Humble
FROM ros:humble-ros-base

# 依存ライブラリを追加
RUN apt-get update && \
    apt-get install -y ros-humble-image-transport ros-humble-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# ワークスペース作成
WORKDIR /ros_ws
COPY ./src ./src

# ビルド
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 実行用エントリ
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

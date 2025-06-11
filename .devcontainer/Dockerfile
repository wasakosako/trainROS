# .devcontainer/Dockerfile
FROM ros:humble-ros-base

# 追加で必要な apt パッケージ
RUN apt-get update && \
    apt-get install -y \
    build-essential python3-pip \
    ros-humble-image-transport \
    ros-humble-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# ワークスペースを作成
WORKDIR /workspaces/ros_ws
RUN mkdir -p src

# デフォルトで /opt/ros の環境を読み込む
ENV BASH_ENV=/opt/ros/humble/setup.bash

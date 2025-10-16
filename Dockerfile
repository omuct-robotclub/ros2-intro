FROM osrf/ros:jazzy-desktop

ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y \
    curl

# 作業ディレクトリ
WORKDIR /home/ros2_ws

#コンポーズでマウント
RUN mkdir -p /ros2_ws/src

# Gzのインストール
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install gz-harmonic -y

# ROS関連のパッケージをインストール
RUN apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz-* \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-dev-tools

RUN rosdep update

# bashrcにソースコマンドを追加
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
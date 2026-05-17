FROM ghcr.io/screamlab/wildbot_base_image:latest

ENV ROS_DISTRO jazzy
ENV SHELL /bin/bash

# A. 系統底層依賴 (最慢，但現在會被永久快取)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        cmake pkg-config swig python3-pip git build-essential \
        ros-${ROS_DISTRO}-demo-nodes-py \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-robot-localization \
        python3-opencv && \
    rm -rf /var/lib/apt/lists/*

# B. AI 庫安裝 (獨立一層，解決 NumPy 衝突)
RUN pip3 install --no-cache-dir --break-system-packages pip setuptools && \
    pip3 install --no-cache-dir --break-system-packages "numpy<2" ultralytics pybullet lap filterpy && \
    pip3 install --no-cache-dir --break-system-packages torch torchvision torchaudio --index-url https://download.pytorch.org/whl/rocm6.1 && \
    python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt'); YOLO('yolov8m.pt')"

# C. YDLidar-SDK 安裝
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/YDLidar-SDK && \
    mkdir -p /tmp/YDLidar-SDK/build && \
    cd /tmp/YDLidar-SDK/build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/YDLidar-SDK

# D. 工作目錄設定 (注意：因為 docker-compose 會掛載這裡，我們只需設定環境)
WORKDIR /workspaces
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /workspaces/install/setup.bash" >> ~/.bashrc

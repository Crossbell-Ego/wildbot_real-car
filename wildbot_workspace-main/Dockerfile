FROM ghcr.io/screamlab/wildbot_base_image:latest
ENV ROS_DISTRO jazzy
COPY . /tmp

WORKDIR /tmp

### list the ros2 packages you want here
#copy the workspaces and install ros packages and dependencies
RUN cp -r ./workspaces/* /workspaces && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        cmake pkg-config swig python3-pip git build-essential \
        ros-${ROS_DISTRO}-demo-nodes-py \
        ros-${ROS_DISTRO}-demo-nodes-cpp && \
    # 安裝 YDLidar-SDK 並指定安裝到 /usr
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/YDLidar-SDK && \
    mkdir -p /tmp/YDLidar-SDK/build && \
    cd /tmp/YDLidar-SDK/build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr .. && \
    make -j$(nproc) && \
    make install && \
    cd /workspaces && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    colcon mixin update && \
    colcon metadata update

# 設定環境變數幫助 CMake 找到 SDK
ENV ydlidar_sdk_DIR=/usr/lib/cmake/ydlidar_sdk

#build the workspace
WORKDIR /workspaces
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -q -y -r --from-paths src --ignore-src && \
    colcon build --symlink-install


RUN rm -rf /tmp/* && \
    rm -rf /temp/* && \
    rm -rf /var/lib/apt/lists/*


ENTRYPOINT ["/ros_entrypoint.bash"]
CMD ["bash", "-l"]

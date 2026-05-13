#!/bin/bash
# launch_shell.sh

set -e

IMAGE_NAME="wildbot_workspace"
DOCKERFILE="Dockerfile"
MARKER=".docker_build_hash"

PROJECT_DIR="$(pwd)"
HOST_WORKSPACE="$PROJECT_DIR"
HOST_CONFIGS="$PROJECT_DIR/docker/compose/configs"
HOST_PHOTOS="/home/robot/桌面/wildbot_photos"

NETWORK_NAME="compose_my_bridge_network"
COMPOSE_PROJECT="compose"
COMPOSE_NETWORK="my_bridge_network"

CONTAINER_NAME="wildbot"

# 建立本機照片資料夾
mkdir -p "$HOST_PHOTOS"

# 計算目前 Dockerfile 的 hash
CURRENT_HASH=$(sha256sum "$DOCKERFILE" | awk '{print $1}')

# 確認是否需要重新 build
NEED_BUILD=false

if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
  echo "[wildbot] Image 不存在，開始 build..."
  NEED_BUILD=true
elif [ ! -f "$MARKER" ]; then
  echo "[wildbot] 找不到 build 記錄，重新 build..."
  NEED_BUILD=true
elif [ "$CURRENT_HASH" != "$(cat "$MARKER")" ]; then
  echo "[wildbot] Dockerfile 有變更，重新 build..."
  NEED_BUILD=true
fi

if [ "$NEED_BUILD" = true ]; then
  docker build -t "$IMAGE_NAME" . || {
    echo "[wildbot] Build 失敗"
    exit 1
  }
  echo "$CURRENT_HASH" > "$MARKER"
  echo "[wildbot] Build 完成"
fi

# 確保 network 存在
if ! docker network inspect "$NETWORK_NAME" &>/dev/null; then
  echo "[wildbot] Network 不存在，建立 $NETWORK_NAME..."
  docker network create \
    --driver bridge \
    --label com.docker.compose.network="$COMPOSE_NETWORK" \
    --label com.docker.compose.project="$COMPOSE_PROJECT" \
    --label com.docker.compose.version="$(docker compose version --short 2>/dev/null || echo 2.0.0)" \
    "$NETWORK_NAME"
fi

# 如果舊 wildbot 還存在，先移除
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo "[wildbot] 發現舊容器 $CONTAINER_NAME，先移除..."
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
fi

# 檢查常用裝置，不存在就提示，但不直接中斷
echo "[wildbot] checking devices..."

if [ ! -e /dev/imu_a9 ]; then
  echo "[wildbot] warning: /dev/imu_a9 不存在，IMU 可能不能用"
fi

if [ ! -e /dev/input/raikiri ]; then
  echo "[wildbot] warning: /dev/input/raikiri 不存在，搖桿可能不能用"
fi

if [ ! -e /dev/ttyACM0 ]; then
  echo "[wildbot] warning: /dev/ttyACM0 不存在，wheel serial 可能不能用"
fi

if [ ! -e /dev/ttyUSB1 ]; then
  echo "[wildbot] warning: /dev/ttyUSB1 不存在，robot arm serial 可能不能用"
fi

# 主機端建立 symlink，存在才建立
if [ -e /dev/ttyACM0 ]; then
  ln -sf /dev/ttyACM0 /dev/usb_wheel
fi

if [ -e /dev/ttyUSB1 ]; then
  ln -sf /dev/ttyUSB0 /dev/usb_robot_arm
fi

echo "[wildbot] starting container..."

docker run -it \
  --name "$CONTAINER_NAME" \
  --rm \
  --privileged \
  --network "$NETWORK_NAME" \
  --group-add 20 \
  --env-file "./docker/compose/.env" \
  -v /dev:/dev \
  -v /dev/input:/dev/input \
  -v "$HOST_WORKSPACE":/workspaces \
  -v "$HOST_CONFIGS":/configs \
  -v "$HOST_PHOTOS":/photos \
  "$IMAGE_NAME" \
  bash -lc "
    echo '[wildbot] container started'

    if [ -e /dev/ttyACM0 ]; then
      ln -sf /dev/ttyACM0 /dev/usb_wheel
      echo '[wildbot] /dev/usb_wheel -> /dev/ttyACM0'
    fi

    if [ -e /dev/ttyUSB1 ]; then
      ln -sf /dev/ttyUSB0 /dev/usb_robot_arm
      echo '[wildbot] /dev/usb_robot_arm -> /dev/ttyUSB0'
    fi

    echo '[wildbot] installing/checking ROS packages...'
    apt update
    apt install -y ros-jazzy-joy ros-jazzy-cv-bridge python3-opencv

    source /opt/ros/jazzy/setup.bash

    echo '[wildbot] checking packages...'
    ros2 pkg list | grep joy || true
    python3 -c 'from cv_bridge import CvBridge; print(\"cv_bridge OK\")' || true
    python3 -c 'import cv2; print(\"opencv\", cv2.__version__)' || true

    echo ''
    echo '[wildbot] ready'
    echo '[wildbot] workspace: /workspaces'
    echo '[wildbot] photos:    /photos'
    echo ''
    echo '常用指令：'
    echo '  source /opt/ros/jazzy/setup.bash'
    echo '  ros2 run joy joy_node --ros-args -p dev:=/dev/input/raikiri'
    echo '  python3 /workspaces/joy_to_base_camera.py'
    echo ''

    exec bash
  "

# 清理 network，如果沒有其他 container 在用
if docker network inspect "$NETWORK_NAME" &>/dev/null; then
  CONNECTED=$(docker network inspect "$NETWORK_NAME" --format '{{len .Containers}}')
  if [ "$CONNECTED" -eq 0 ]; then
    echo "[wildbot] 沒有其他 container 使用 $NETWORK_NAME，移除..."
    docker network rm "$NETWORK_NAME"
  else
    echo "[wildbot] $NETWORK_NAME 仍有 $CONNECTED 個 container 在使用，保留"
  fi
fi

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +local:docker

docker rm -f velmobil_simulation 2>/dev/null || true

docker run -d \
    --name velmobil_simulation \
    --hostname velmobil_simulation \
    --gpus all \
    --network host \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __VK_LAYER_NV_optimus=NVIDIA_only \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$SCRIPT_DIR":/root/ws/src/VELMOBIL_Troszczynski_Piecha \
    velmobil_simulation:latest
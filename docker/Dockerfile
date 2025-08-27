
# --- Jetson + ROS base (JetPack 6.2 / L4T r36.4.0) ---
ARG L4T_TAG=l4t-r36.4.0
FROM dustynv/ros:humble-desktop-${L4T_TAG}

# Non-interactive apt + lean pip (no cache)
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_NO_CACHE_DIR=1
ENV PIP_DEFAULT_TIMEOUT=120

# Use ARM (ports) mirrors; drop stale ROS2 apt lists; initial update
RUN set -eux; \
    sed -i 's|http://archive.ubuntu.com|http://ports.ubuntu.com|g; s|http://security.ubuntu.com|http://ports.ubuntu.com|g' /etc/apt/sources.list; \
    rm -f /etc/apt/sources.list.d/ros2*.list || true; \
    apt-get update

# --- System deps (editors, build tools) + runtime libs for OpenCV/GStreamer ---
RUN apt-get install -y --no-install-recommends \
      python3-pip python3-dev build-essential \
      nano vim less ca-certificates curl gnupg \
      libgl1 libglib2.0-0 \
      # numpy/packaging/ninja from apt (ARM-ready; avoids pip source builds)
      python3-numpy python3-packaging python3-typing-extensions ninja-build \
      # helpful for camera/video pipelines
      gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav \
  && rm -rf /var/lib/apt/lists/*

# --- PyTorch (Jetson JP v61 wheel; pinned because v61 lists only 2.5 today) ---
# NOTE: Torch >=2.4 may require libcusparseLt at runtime. If your host JetPack lacks it,
# you’ll need to install the host package (libcusparselt*) or switch base to l4t-ml.
ENV NV_TORCH_URL_BASE=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/
ENV TORCH_WHL=torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
RUN set -eux; \
    curl -fSL "${NV_TORCH_URL_BASE}${TORCH_WHL}" -o "/tmp/${TORCH_WHL}"; \
    python3 -m pip install --no-cache-dir --no-deps "/tmp/${TORCH_WHL}"; \
    rm -f "/tmp/${TORCH_WHL}"

# --- OpenCV with GStreamer from apt (avoid PyPI opencv on Jetson) ---
RUN apt-get update && apt-get install -y --no-install-recommends python3-opencv && rm -rf /var/lib/apt/lists/*

# --- Force PyPI for userland Python deps (Dusty base may set a Jetson mirror) ---
ENV PIP_INDEX_URL=https://pypi.org/simple
ENV PIP_EXTRA_INDEX_URL=

# --- Ultralytics (YOLOv8) WITHOUT deps (prevents pulling opencv-python), plus light deps ---
RUN python3 -m pip install --no-cache-dir --no-deps \
      ultralytics==8.3.0 pillow pyyaml tqdm requests psutil

# --- (Intentionally no build-time imports of torch/cv2) ---
# CUDA libs are injected at runtime by the NVIDIA container runtime;
# importing torch during *build* would fail on Jetson.

# --- Workspace wiring ---
WORKDIR /workspace
COPY . /workspace

# Ensure entrypoint is executable
RUN chmod +x /workspace/docker/entrypoint.sh

# Default entrypoint (hardened: it should source ROS if present and exec "$@")

ENTRYPOINT ["/workspace/docker/entrypoint.sh"]

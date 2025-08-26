FROM ros:humble

# Keep installs non-interactive and pip tidy
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_NO_CACHE_DIR=1

# --- System deps: pip, editors, numpy headers, OpenCV GUI libs ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev build-essential \
    python3-numpy \
    nano vim less \
    libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# --- Python stack: Torch (CPU), then stable vision libs (versions that play nice together) ---
RUN python3 -m pip install --upgrade pip \
 && python3 -m pip install \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu \
 && python3 -m pip install \
    numpy==1.26.4 \
    opencv-python-headless==4.11.0.86 \
    ultralytics==8.3.0

# --- Sanity check so you know versions are baked in ---
RUN python3 - <<'PY'
import torch, cv2, numpy, ultralytics
print("torch", torch.__version__,
      "opencv", cv2.__version__,
      "numpy", numpy.__version__,
      "ultralytics", ultralytics.__version__)
PY


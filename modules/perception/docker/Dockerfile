ARG BASE_IMAGE=autoware_backup:latest
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-can \
        ros-humble-vision-msgs \
    && python3 -m pip install --no-cache-dir \
        "numpy<2" \
        "onnxruntime>=1.17,<2" \
    && rm -rf /var/lib/apt/lists/*

# A Jetson deployment may replace onnxruntime with the JetPack-compatible
# GPU wheel. The application selects CUDA only when that provider is present.

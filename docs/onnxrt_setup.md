# Building ONNXRuntime GPU from Source
## Jetson Nano / JetPack 4.6.1 / Python 3.10 / TensorRT 8.0.1

This document describes the steps required to build ONNXRuntime 1.10.0 with CUDA and TensorRT support for deployment on a Jetson Nano running a Ubuntu 22.04 Docker container with ROS2 Humble.

---

## System Configuration

| Component | Version |
|---|---|
| Hardware | Jetson Nano (Tegra X1, SM 5.3) |
| JetPack | 4.6.1 (L4T R32.6.1) |
| Host OS | Ubuntu 18.04 |
| Container OS | Ubuntu 22.04 |
| CUDA | 10.2 |
| cuDNN | 8.2 |
| TensorRT | 8.0.1 |
| Python (container) | 3.10 |
| ONNXRuntime | 1.10.0 |

### Why ORT 1.10.0 specifically

- ORT 1.13.x and newer use CUDA 11+ cuBLAS APIs (`CUBLAS_TF32_TENSOR_OP_MATH`, `cublasComputeType_t`) not present in CUDA 10.2
- ORT 1.11.x and newer use `nvinfer1::ScatterMode` which was introduced in TRT 8.2 — not available on this Nano
- ORT 1.10.0 is the last version whose CUDA provider and bundled `onnx-tensorrt` submodule are compatible with TRT 8.0.1 and CUDA 10.2

---

## Prerequisites

### 1. Verify NVIDIA container runtime is installed on the host

```bash
# On the Nano host
which nvidia-container-runtime
dpkg -l | grep nvidia-container
```

If missing, install it and register with Docker:

```bash
sudo apt-get install -y nvidia-container-runtime

sudo nano /etc/docker/daemon.json
# Add:
# {
#     "runtimes": {
#         "nvidia": {
#             "path": "nvidia-container-runtime",
#             "runtimeArgs": []
#         }
#     },
#     "default-runtime": "nvidia"
# }

sudo systemctl restart docker
docker info | grep -i runtime
```

### 2. Mount required host libraries into the container

Update `start_ros2_container.sh` to include the following volume mounts:

```bash
#!/bin/bash
docker run -d --name ros2_humble --restart unless-stopped \
  -u ros \
  --net=host \
  --privileged \
  --runtime nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /dev:/dev --device-cgroup-rule='c *:* rmw' \
  -v /usr/lib/aarch64-linux-gnu/libEGL.so.1:/usr/lib/aarch64-linux-gnu/libEGL.so.1:ro \
  -v /usr/local/cuda-10.2:/usr/local/cuda-10.2:ro \
  -v ~/X1_ROS2_ws:/X1_ROS2_ws \
  -v ~/robot_ws:/robot_ws \
  ros2_humble_img bash -c "sudo mkdir -p /run/sshd && sudo /usr/sbin/sshd -D"
```

The full `/usr/local/cuda-10.2` mount is required for both the CUDA libraries (runtime) and headers (build time).

### 3. Patch the CUDA 10.2 GCC version check on the host

CUDA 10.2 rejects GCC > 8, but ORT 1.10.0 requires GCC >= 9. Patch the check on the host (change takes effect immediately in the container via the bind mount):

```bash
# On the Nano host
sudo nano /usr/local/cuda-10.2/targets/aarch64-linux/include/crt/host_config.h
```

Find line 138 and change:
```c
#if __GNUC__ > 8
#error -- unsupported GNU version! gcc versions later than 8 are not supported!
```
To:
```c
#if __GNUC__ > 9
#error -- unsupported GNU version! gcc versions later than 9 are not supported!
```

---

## Build Steps (inside the container)

### Step 1 — Install build dependencies

```bash
sudo apt-get install -y \
    build-essential \
    libopenblas-dev \
    python3-dev \
    python3-pip \
    wget \
    curl \
    unzip \
    tmux \
    software-properties-common

pip3 install wheel
```

### Step 2 — Install GCC 9

GCC 9 satisfies both ORT's minimum (>= 9) and CUDA 10.2's patched maximum (<= 9). GCC 8 is not available in Ubuntu 22.04 repos and ORT rejects it anyway.

```bash
# Add Ubuntu 20.04 focal repos temporarily
echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports focal main universe" \
    | sudo tee /etc/apt/sources.list.d/focal.list

sudo apt-get update
sudo apt-get install -y gcc-9 g++-9

# Remove focal repo immediately to avoid package conflicts
sudo rm /etc/apt/sources.list.d/focal.list
sudo apt-get update

# Verify
gcc-9 --version
g++-9 --version
```

### Step 3 — Install cmake 3.26.6

Ubuntu 22.04 ships cmake 3.22 which is below ORT 1.10.0's minimum of 3.26. cmake 4.x breaks ORT's submodule dependency syntax so use exactly 3.26.6.

```bash
wget https://github.com/Kitware/CMake/releases/download/v3.26.6/cmake-3.26.6-linux-aarch64.tar.gz
tar -xzf cmake-3.26.6-linux-aarch64.tar.gz
sudo mv cmake-3.26.6-linux-aarch64 /opt/cmake-3.26.6

export PATH=/opt/cmake-3.26.6/bin:$PATH
echo 'export PATH=/opt/cmake-3.26.6/bin:$PATH' >> ~/.bashrc

cmake --version
# Expected: cmake version 3.26.6
```

### Step 4 — Set environment variables

```bash
export CUDA_HOME=/usr/local/cuda-10.2
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/targets/aarch64-linux/lib:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH

# Make permanent
echo 'export CUDA_HOME=/usr/local/cuda-10.2' >> ~/.bashrc
echo 'export PATH=$CUDA_HOME/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-10.2/targets/aarch64-linux/lib:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH' >> ~/.bashrc
```

### Step 5 — Clone ORT 1.10.0

```bash
cd ~
git clone --recursive https://github.com/microsoft/onnxruntime.git
cd onnxruntime
git checkout v1.10.0
git submodule update --init --recursive

# Verify
git log --oneline -1
```

### Step 6 — Pre-download eigen

ORT 1.10.0's FetchContent-based eigen download is unreliable with cmake 3.26. Pre-download it manually using the exact URL and commit from `cmake/deps.txt`:

```bash
wget "https://gitlab.com/libeigen/eigen/-/archive/e7248b26a1ed53fa030c5c459f7ea095dfd276ac/eigen-e7248b26a1ed53fa030c5c459f7ea095dfd276ac.zip" \
    -O /tmp/eigen.zip

mkdir -p /tmp/eigen_tmp
unzip /tmp/eigen.zip -d /tmp/eigen_tmp
mkdir -p /tmp/eigen
mv /tmp/eigen_tmp/eigen-e7248b26a1ed53fa030c5c459f7ea095dfd276ac/* /tmp/eigen/
rm -rf /tmp/eigen_tmp

# Verify
ls /tmp/eigen/Eigen/
```

### Step 7 — Build

Start a tmux session first to survive disconnections:

```bash
tmux new -s ort_build
```

Run the build (expect 4-6 hours on the Nano):

```bash
cd ~/onnxruntime

CC=gcc-9 CXX=g++-9 ./build.sh \
    --config Release \
    --build_shared_lib \
    --use_cuda \
    --cuda_home /usr/local/cuda-10.2 \
    --cudnn_home /usr/lib/aarch64-linux-gnu \
    --use_tensorrt \
    --tensorrt_home /usr/src/tensorrt \
    --build_wheel \
    --skip_tests \
    --parallel 2 \
    --cmake_extra_defines \
        CMAKE_CUDA_COMPILER=/usr/local/cuda-10.2/bin/nvcc \
        CMAKE_CUDA_ARCHITECTURES=53 \
        CMAKE_C_COMPILER=gcc-9 \
        CMAKE_CXX_COMPILER=g++-9 \
        CMAKE_CUDA_HOST_COMPILER=g++-9 \
        CUDA_INCLUDE_DIRS=/usr/local/cuda-10.2/targets/aarch64-linux/include \
        CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2 \
        onnxruntime_USE_CUDA=ON \
        CMAKE_BUILD_TYPE=Release \
        onnxruntime_USE_PREINSTALLED_EIGEN=ON \
        eigen_SOURCE_PATH=/tmp/eigen
```

Key build flags explained:

- `CMAKE_CUDA_HOST_COMPILER=g++-9` — forces nvcc to use GCC 9 as host compiler instead of defaulting to system GCC 11
- `CMAKE_CUDA_ARCHITECTURES=53` — targets Maxwell SM 5.3 (Jetson Nano's GPU) specifically
- `CUDA_INCLUDE_DIRS` — explicitly set because ORT 1.10.0 uses the older FindCUDA module which fails to locate headers on aarch64's non-standard layout
- `onnxruntime_USE_PREINSTALLED_EIGEN=ON` — bypasses FetchContent for eigen, using the pre-downloaded copy instead
- `--parallel 2` — limits parallel jobs to prevent OOM on the Nano's 4GB shared RAM

If the build fails at wheel packaging with `No module named 'wheel'`, install it and rerun the same command without cleaning the build directory:

```bash
pip3 install wheel
# Rerun build command — compiled objects are cached, only wheel packaging reruns
```

### Step 8 — Install the wheel

```bash
cd ~
pip3 install ~/onnxruntime/build/Linux/Release/dist/onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl
```

Save the wheel to the mounted volume for persistence across container rebuilds:

```bash
cp ~/onnxruntime/build/Linux/Release/dist/onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl \
   /X1_ROS2_ws/
```

### Step 9 — Verify

```bash
cd ~  # Important: must be outside the onnxruntime source directory
python3 -c "
import onnxruntime as ort
print('ORT version:', ort.__version__)
print('Available providers:', ort.get_available_providers())
"
```

Expected output:
```
ORT version: 1.10.0
Available providers: ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
```

---

## Dockerfile Integration

Add the following to the Dockerfile to bake the wheel into the image. Place the `ENV` instructions near the top (before any `USER` switch) and the `COPY`/`RUN` instructions after `USER $USERNAME`:

```dockerfile
# Near the top of the Dockerfile — sets paths for all processes including ROS2 nodes
ENV LD_LIBRARY_PATH=/usr/local/cuda-10.2/targets/aarch64-linux/lib:/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}
ENV PATH=/usr/local/cuda-10.2/bin:${PATH}

# After USER $USERNAME — install the pre-built ORT wheel
COPY onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl /tmp/
RUN pip3 install /tmp/onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl && \
    rm /tmp/onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl
```

Place the wheel file in the same directory as the Dockerfile before building:

```bash
cp /X1_ROS2_ws/onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl /path/to/dockerfile/dir/
docker build -t ros2_humble_img /path/to/dockerfile/dir/
```

> **Note:** Do not commit the `.whl` file to Git. Add `*.whl` to `.gitignore`.
> The wheel is architecture and CUDA version specific — it only works on Jetson Nano with JetPack 4.6.x.
> If the Nano is reflashed or the JetPack version changes, rebuild ORT from source following this guide.\
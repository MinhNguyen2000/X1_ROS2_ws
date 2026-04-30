# Base ROS2 Humble image
FROM ros:humble-ros-base-jammy

# Install the system tools
RUN apt-get update && apt-get install -y \
  iproute2 \
  iputils-ping \
  joystick \
  net-tools \
  nano \
  libboost-dev \
 && rm -rf /var/lib/apt/lists/*

# Install some base ROS2 nodes for communication testing
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN apt update && apt install -y python3-pip \
    && rm -rf /var/lib/apt/list/*

# Build the ydlidar SDK
WORKDIR /
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
  && cd YDLidar-SDK \
  && mkdir build && cd build \
  && cmake .. \
  && make -j4 \
  && sudo make install

# Update the shared lib cache
#RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/ydlidar.conf && ldconfig

# Install packages for Orbbec Astra Pro camera
RUN apt-get update && apt-get install -y \
  libgflags-dev \
  nlohmann-json3-dev  \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-image-publisher \
  ros-${ROS_DISTRO}-camera-info-manager \
  ros-${ROS_DISTRO}-diagnostic-updater \
  ros-${ROS_DISTRO}-diagnostic-msgs \
  ros-${ROS_DISTRO}-statistics-msgs \
  ros-${ROS_DISTRO}-backward-ros \
  libdw-dev \
&& rm -rf /var/lib/apt/lists/*

# Source the ROS2 underlay
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set shared library path and cuda tools executable paths for CUDA inference
ENV LD_LIBRARY_PATH=/usr/local/cuda-10.2/targets/aarch64-linux/lib:/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}
ENV PATH=/usr/local/cuda-10.2/bin:${PATH}

#COPY /home/jetson/OrbbecSDK/lib/arm64/libOrbbecSDK.so /usr/local/lib
#RUN ldconfig

## Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Install and configure SSH server  <-- insert here, still as root
RUN apt-get update && apt-get install -y openssh-server \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /run/sshd \
    && sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config \
    && sed -i 's/#PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config

RUN sudo ssh-keygen -A 

USER $USERNAME

# Set password (for ssh)
RUN echo "$USERNAME:yahboom" | sudo chpasswd

# Environment variables (for ROS communication for example)
COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/bin/bash","/ros_entrypoint.sh"]
RUN echo "source /ros_entrypoint.sh" >> ~/.bashrc

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Add user to dialout group to access the serial port
RUN sudo usermod -aG dialout $USERNAME

# Update all packages
RUN sudo apt update -y

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /X1_ROS2_ws/install/setup.bash" >> ~/.bashrc

# Set CUDA library paths for interactive SSH sessions
RUN echo 'export LD_LIBRARY_PATH=/usr/local/cuda-10.2/targets/aarch64-linux/lib:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH' >> ~/.bashrc

# Install ONNXRuntimeGPU wheel built for JetPack 4.6 + CUDA 10.2 + Python 3.10
COPY onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl /tmp/
RUN sudo pip3 install /tmp/onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl && \
    sudo rm /tmp/onnxruntime_gpu-1.10.0-cp310-cp310-linux_aarch64.whl

# Install Rosmaster_lib
RUN sudo pip3 install pyserial

COPY rosmaster_driver_install /tmp/rosmaster_driver_install
RUN cd /tmp/rosmaster_driver_install && sudo python3 setup.py install

# Install Python stuff
RUN pip install "numpy<2" \
    matplotlib \
    opencv-python \
    stable-baselines3

WORKDIR /X1_ROS2_ws
USER root
# Optional: copy the workspace into the container
# COPY . /root/robot_ws

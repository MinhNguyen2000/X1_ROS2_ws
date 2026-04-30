# Script to start a ros:humble docker container

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

# -v /usr/lib/aarch64-linux-gnu/libEGL.so.1:/usr/lib/aarch64-linux-gnu/libEGL.so.1:ro \

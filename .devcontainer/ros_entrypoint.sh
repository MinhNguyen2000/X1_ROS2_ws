#!/bin/bash

# Script to set environment variables

export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
export ROS_IP=$(hostname -I | awk '{print $1}')
export LIDAR_TYPE=x3

echo "----------"
echo -e "ROS_DOMAIN_ID:\033[32m$ROS_DOMAIN_ID\033[0m | ROS_IP:\033[32m$ROS_IP\033[0m"
echo -e "LIDAR_TYPE: \033[32m$LIDAR_TYPE\033[0m"
echo "----------"
exec "$@"
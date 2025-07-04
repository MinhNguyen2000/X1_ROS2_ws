#!/bin/bash

# Script to set environment variables

export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
export ROS_IP=$(hostname -I | awk '{print $1}')

echo "----------"
echo -e "ROS_DOMAIN_ID:$ROS_DOMAIN_ID"
echo -e "ROS_IP:$ROS_IP"
echo "----------"
exec "$@"
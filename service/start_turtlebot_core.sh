#!/usr/bin/env bash
sudo -H -u psawaffle1 bash -c "source /opt/ros/kinetic/setup.bash \
&& export ROS_MASTER_URI=http://192.168.255.141:11311 \
&& export ROS_HOSTNAME=192.168.255.141 \
&& echo \"======== $(date) ========\" >> ~/.ros/turtlebot-roscore.log \
&& roscore >> ~/.ros/turtlebot-roscore.log"


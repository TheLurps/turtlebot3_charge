#!/usr/bin/env bash
sudo -H -u psawaffle1 bash -c "source /home/psawaffle1/catkin_ws/devel/setup.bash \
&& export ROS_MASTER_URI=http://192.168.255.141:11311 \
&& export ROS_HOSTNAME=192.168.255.141 \
&& echo \"======== $(date) ========\" >> ~/.ros/turtlebot_battery.log \
&& roslaunch --wait -t 10 turtlebot3_charge battery.launch >> ~/.ros/turtlebot_battery.log"


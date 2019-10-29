#!/usr/bin/env bash
sudo -H -u psawaffle1 bash -c "source /home/psawaffle1/catkin_ws/devel/setup.bash \
&& export ROS_MASTER_URI=http://192.168.255.141:11311 \
&& export ROS_HOSTNAME=192.168.255.141 \
&& export TURTLEBOT3_MODEL=waffle \
&& echo \"======== $(date) ========\" >> ~/.ros/turtlebot_nav.log \
&& roslaunch --wait -t 10 turtlebot3_charge nav.launch >> ~/.ros/turtlebot_nav.log"


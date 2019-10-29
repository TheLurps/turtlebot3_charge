# turtlebot3_charge

## Development and Implementation of a LIDAR-based Method for approaching a Charging Station with a Turtlebot 3
This project will consider the development and implementation of a LIDAR-based method for approaching a Charging Station using the TurtleBot 3 platform. The requirement of charging an autonomous system during operation is caused by the limited capacity of storing energy within the system. Therefore, it is necessary to approach a charging station based on information given by embedded sensors. A widely used sensor that should be capable of achieving this is a LIDAR. The capabilities of recognizing materials of different reflectance will be evaluated and compared. Reflective tape is chosen, since it is most efficient. A software solution for localizing and navigationn towards a marked charging station will be developed and evaluated.

### Requirements

### Installation
```shell
cd ~/catkin_ws/src
git clone https://github.com/yzrobot/pose_publisher.git
git clone https://github.com/TheLurps/turtlebot3_charge.git
cd turtlebot3_charge
pip install -r requirements.txt
sudo cp -v service/*.sh /usr/local/bin/
sudo cp -v service/*.service /etc/systemd/system/
sudo systemctl enable turtlebot_core.service
sudo systemctl enable turtlebot_bringup.service
sudo systemctl enable turtlebot_slam.service
sudo systemctl enable turtlebot_nav.service
sudo reboot
```

### Usage

#### Start package
Login into TurtleBot via SSH, then
```shell
roslaunch turtlebot3_charge sleeper.launch
```

#### Launch rviz locally
```shell
roslaunch turtlebot3_charge rviz.launch
```

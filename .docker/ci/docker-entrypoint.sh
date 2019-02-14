#!/bin/bash
cd /home/root/ros2_ws/src/hrim
sudo pip3 install -r requirements.txt

hrim generate --platform ros2 all

/bin/bash -c "source /opt/ros/crystal/setup.bash && colcon build --merge-install"
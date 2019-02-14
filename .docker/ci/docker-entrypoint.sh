#!/bin/bash
cd /home/root/ros2_ws/src/hrim
python3 hrim.py generate --platform ros2 all

/bin/bash -c "source /opt/ros/crystal/setup.bash && colcon build --merge-install"
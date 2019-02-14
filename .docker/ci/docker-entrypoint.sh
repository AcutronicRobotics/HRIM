#!/bin/bash
ls /home/root/ros2_ws/src/hrim erlerobot/hrim

/bin/bash -c "source /opt/ros/crystal/setup.bash && colcon build --merge-install"
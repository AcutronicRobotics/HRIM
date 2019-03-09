#!/bin/bash
set -e
cd /home/root/ros2_ws/src/hrim/installator
pip3 install -r requirements.txt
python3 setup.py install
cd ..

pylint --rcfile linter/.pylintrc installator/hrim/
result=$?
if [ $result -ne 0 ]; then
  echo "lint error!!"
  exit 123
fi

hrim generate --platform ros2 all
cd generated

/bin/bash -c "source /opt/ros/crystal/setup.bash && colcon build --merge-install"

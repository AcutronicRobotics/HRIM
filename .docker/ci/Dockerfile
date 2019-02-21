FROM ros:crystal-ros-base-bionic

MAINTAINER Asier Bilbao <asier@erlerobotics.com>

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-crystal-action-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install astroid==1.4.4 pylint==1.5.2

RUN mkdir -p ros2_ws/src
WORKDIR ros2_ws

COPY ./docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]
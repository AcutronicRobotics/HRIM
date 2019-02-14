FROM ros:crystal-ros-base-bionic

MAINTAINER Asier Bilbao <asier@erlerobotics.com>

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-crystal-action-msgs \
    && pip3 install lxml \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ros2_ws/src
WORKDIR ros2_ws

COPY ./docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]
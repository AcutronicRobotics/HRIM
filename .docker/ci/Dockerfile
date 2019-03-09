FROM ubuntu:bionic

MAINTAINER Asier Bilbao <asier@erlerobotics.com>

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS2_DISTRO crystal
WORKDIR /home/root/ros2_ws
COPY ./docker-entrypoint.sh /

RUN \
      echo 'Etc/UTC' > /etc/timezone \
      && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
      && apt-get update -qq && apt-get install -qq -y tzdata dirmngr gnupg2 lsb-release curl \
      # setup ros2 keys
      && curl http://repo.ros2.org/repos.key | apt-key add - \
      # setup sources.list
      && echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
      && apt-get update -qq && apt-get install -qq -y \
         build-essential \
         cmake \
         git \
         python3-colcon-common-extensions \
         python3-pip \
         python3-vcstool \
         wget \
         libopensplice69 \
         ros-$ROS2_DISTRO-ros-base \
         ros-$ROS2_DISTRO-action-msgs \
         && rm -rf /var/lib/apt/lists/* \
         && pip3 install astroid==1.4.4 pylint==1.5.2 \
         && mkdir -p ros2_ws/src

ENTRYPOINT ["/docker-entrypoint.sh"]

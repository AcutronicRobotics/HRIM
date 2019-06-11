FROM ros:dashing

LABEL maintainer="Asier Bilbao <asier at erlerobotics dot com>"

ENV ROS2_DISTRO dashing
WORKDIR /home/root/ros2_ws
COPY ./docker-entrypoint.sh /

RUN \
      apt update -qq && apt install pep8 -y \
      && pip3 install dfusexml \
      && mkdir -p ros2_ws/src

ENTRYPOINT ["/docker-entrypoint.sh"]

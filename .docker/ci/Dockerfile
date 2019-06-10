FROM ros:dashing

LABEL maintainer="Asier Bilbao <asier at erlerobotics dot com>"

ENV ROS2_DISTRO dashing
WORKDIR /home/root/ros2_ws
COPY ./docker-entrypoint.sh /

RUN \
      pip3 install astroid==1.4.4 pylint==1.5.2 \
      && mkdir -p ros2_ws/src

ENTRYPOINT ["/docker-entrypoint.sh"]

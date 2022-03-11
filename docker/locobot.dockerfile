# syntax=docker/dockerfile:experimental

FROM ubuntu:focal

RUN echo 'America/New_York' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/America/New_York /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

### second stage ###
ENV LC_ALL C.UTF-8
ENV LANG C.UTF-8

# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
  ln -s /bin/bash /bin/sh

# install basic system stuff
COPY ./install_scripts/install_basic.sh /tmp/install_basic.sh
RUN chmod +x /tmp/install_basic.sh
RUN /tmp/install_basic.sh


# install ROS stuff
ENV ROS_DISTRO noetic



COPY ./install_scripts/install_python.sh /tmp/install_python.sh
RUN chmod +x /tmp/install_python.sh
RUN /tmp/install_python.sh

ARG DEBIAN_FRONTEND=noninteractive

# create catkin workspace


COPY ./install_scripts/install_realsense.sh /tmp/install_realsense.sh
RUN chmod +x /tmp/install_realsense.sh
RUN /tmp/install_realsense.sh

COPY ./install_scripts/install_ros.sh /tmp/install_ros.sh
RUN chmod +x /tmp/install_ros.sh
RUN /tmp/install_ros.sh

# bootstrap rosdep
RUN rosdep init 
RUN rosdep update


ENV CATKIN_WS=/root/catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash


COPY ./install_scripts/install_pytorch.sh /tmp/install_pytorch.sh
RUN chmod +x /tmp/install_pytorch.sh
RUN /tmp/install_pytorch.sh


COPY ./install_scripts/install_dynamixel.sh /tmp/install_dynamixel.sh
RUN chmod +x /tmp/install_dynamixel.sh
RUN /tmp/install_dynamixel.sh

# copy local requirements file for pip install python deps
ENV IMPROB /workspace
RUN mkdir ${IMPROB}


COPY install_scripts/install_py_pkgs.sh /tmp/install_py_pkgs.sh
RUN chmod +x /tmp/install_py_pkgs.sh
RUN /tmp/install_py_pkgs.sh

COPY ./install_scripts/install_libcreate.sh /tmp/install_libcreate.sh
RUN chmod +x /tmp/install_libcreate.sh
RUN /tmp/install_libcreate.sh


COPY ./install_scripts/install_locobot.sh /tmp/install_locobot.sh
RUN chmod +x /tmp/install_locobot.sh
RUN /tmp/install_locobot.sh

COPY install_scripts/install_py_pkgs2.sh /tmp/install_py_pkgs2.sh
RUN chmod +x /tmp/install_py_pkgs2.sh
RUN /tmp/install_py_pkgs2.sh

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/create_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "source /root/locobot_ws/devel/setup.bash" >> /root/.bashrc


WORKDIR /
# Exposing the ports
EXPOSE 11311


# setup entrypoint  
COPY ./entrypoint.sh /

ENTRYPOINT ["./entrypoint.sh"]
CMD ["bash"]





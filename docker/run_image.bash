IMAGE=ros:noetic-ros-core-focal
docker run -it\
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PWD/../../:/workspace/" \
    -v /dev:/dev \
    --privileged \
    --net=host \
    ${IMAGE}


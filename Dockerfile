FROM ros:jazzy

LABEL maintainer="SamukoThings <samukothings@gmail.com>"

SHELL ["/bin/bash", "-c"]

# Add the Kisak Mesa PPA repository for updated graphics drivers
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    DEBIAN_FRONTEND=noninteractive add-apt-repository -y ppa:kisak/kisak-mesa

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Install core Mesa packages and graphics utilities
RUN apt-get install -y \
        mesa-utils \
        libgl1-mesa-dri && \
    rm -rf /var/lib/apt/lists/*

# Set up the XDG runtime directory required by modern Qt6 GUI apps
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p $XDG_RUNTIME_DIR && \
    chmod 700 $XDG_RUNTIME_DIR

RUN mkdir -p /root/ros_ws/src

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY ./robot_description /root/ros_ws/src/helloworld_navigation/robot_description
COPY ./robot_simulation /root/ros_ws/src/helloworld_navigation/robot_simulation
COPY ./robot_navigation /root/ros_ws/src/helloworld_navigation/robot_navigation

WORKDIR /root/ros_ws

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/*

COPY ./entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["bash"]


# (only once)
# xhost +local:docker

# cd $HOME/ros_ws/src/helloworld_navigation/
# docker compose up --build
# docker compose up
# docker exec -it hello_nav_container bash
# docker compose down --remove-orphans

# docker run -it \
#   --net=host \
#   --env="DISPLAY=$DISPLAY" \
#   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#   your_image_name
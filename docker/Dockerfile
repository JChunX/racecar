FROM ros:humble

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       libfmt-dev \
                       librange-v3-dev \
                       libwebsocketpp-dev \
                       nlohmann-json3-dev \
                       ros-humble-rviz2 \
                       openssh-server
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .

RUN mkdir -p sim_ws/src

# foxglove bridge
RUN mkdir sim_ws/src/ros-foxglove-bridge
RUN git clone https://github.com/foxglove/ros-foxglove-bridge /sim_ws/src/ros-foxglove-bridge

# ros2 gym bridge
RUN mkdir sim_ws/src/f1tenth_gym_ros
COPY car_ws/src/f1tenth_gym_ros /sim_ws/src/f1tenth_gym_ros

# build
RUN source /opt/ros/humble/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

# entrypoint script
COPY racecar /bin/racecar
RUN chmod +x /bin/racecar

WORKDIR '/sim_ws'

ENV ROS_DOMAIN_ID=42
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

RUN mkdir /vehicle_logs

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .

RUN mkdir -p car_ws/src

# copy car_ws/src to car_ws/src
COPY car_ws/src /car_ws/src

# build
RUN source /opt/ros/humble/setup.bash && \
    cd car_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

# convenience script
COPY racecar /bin/racecar
RUN chmod +x /bin/racecar

# ros & workspace setup
COPY source_sim /source_sim
RUN chmod +x /source_sim

WORKDIR '/car_ws'

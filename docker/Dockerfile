FROM ros:humble-ros-core-jammy
MAINTAINER Toya Takahashi <toyatakahashi522@gmail.com>

# install bootstrap tools
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \ 
    build-essential \
    python3-colcon-common-extensions

## bootstrap rosdep
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# install ros2 packages
RUN apt-get update && apt-get install -y ros-humble-ros-base

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Create a sudoer user "arcturus" with arcturus as a password
RUN useradd arcturus -G sudo -m \
    && echo "arcturus:arcturus" | chpasswd
WORKDIR /home/arcturus

# Set up timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

# Install required utilities
RUN apt update \
&& apt install -y \
    vim
    #gnupg2 \
    #python3-dbg \
    #python3-setuptools \
    #ruby \
    #sudo

# Set up locale
RUN sudo apt update && sudo apt install locales \
&& sudo locale-gen en_US en_US.UTF-8 \
&& sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ARG ROSDIST=humble

# Install required packages
RUN sudo apt update && apt upgrade -y
RUN sudo apt install -y --no-install-recommends \
    subversion g++ cmake xterm libfltk1.3-dev freeglut3-dev libpng-dev libjpeg-dev libxft-dev libxinerama-dev libtiff5-dev \
    protobuf-compiler libb64-dev ros-humble-diagnostic-updater lsb-release wget gnupg libprotobuf-dev

# Set up AllSeaingVehicle2
RUN git clone https://github.com/ArcturusNavigation/AllSeaingVehicle2.git
RUN cd /home/arcturus/AllSeaingVehicle2 \
&& git submodule update --init --recursive src/robot_localization \
&& rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
&& protoc --cpp_out=./src/protobuf_client_ros2/include/protobuf_client ./src/protobuf_client_ros2/include/protobuf_client/gateway.proto \
&& source /opt/ros/humble/setup.bash \
&& colcon build --merge-install

# Setup environment
ENV HOME=/home/arcturus
WORKDIR /home/arcturus
ENV PATH=/home/arcturus/moos-ivp/bin:$PATH
ENV PATH=/home/arcturus/moos-ivp/scripts:$PATH
ENV PATH=/home/arcturus/moos-ivp-arcturus/bin:$PATH
ENV PATH=/home/arcturus/moos-ivp-arcturus/scripts:$PATH
ENV IVP_BEHAVIOR_DIRS=/home/arcturus/moos-ivp/lib

# Set up MOOS-IvP
RUN svn co https://oceanai.mit.edu/svn/moos-ivp-aro/trunk/ moos-ivp
RUN cd /home/arcturus/moos-ivp && ./build-moos.sh && ./build-ivp.sh
RUN git clone https://github.com/ArcturusNavigation/moos-ivp-arcturus.git
RUN cd /home/arcturus/moos-ivp-arcturus && ./build.sh

# Copy ros_entrypoint
COPY ./ros_entrypoint.sh /

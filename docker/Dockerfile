FROM ros:humble-ros-core-jammy
MAINTAINER Toya Takahashi <toyatakahashi522@gmail.com>

# install bootstrap tools
RUN apt update && apt install -y \
    git python3-rosdep2 build-essential python3-colcon-common-extensions vim sudo

# install ros2 packages
RUN sudo apt update && sudo apt install -y ros-humble-ros-base

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Create a sudoer user "arcturus" with arcturus as a password
RUN useradd arcturus -G sudo -m \
&& echo "arcturus:arcturus" | chpasswd
ENV HOME=/home/arcturus
WORKDIR /home/arcturus

# Set up timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone \
&& ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

# Set up locale
RUN apt update && apt install locales \
&& sudo locale-gen en_US en_US.UTF-8 \
&& sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install required packages
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install -y --no-install-recommends \
    subversion g++ cmake xterm libfltk1.3-dev freeglut3-dev libpng-dev libjpeg-dev libxft-dev libxinerama-dev libtiff5-dev \
    protobuf-compiler libb64-dev ros-humble-diagnostic-updater lsb-release wget gnupg libprotobuf-dev

# Setup all_seaing_vehicle
RUN mkdir -p dev_ws/src
WORKDIR /home/arcturus/dev_ws/src
RUN git clone https://github.com/ArcturusNavigation/all_seaing_vehicle
RUN git clone https://github.com/mikedef/protobuf_client_ros2
RUN git clone https://github.com/mikedef/protobuf_client_interfaces
RUN git clone https://github.com/cra-ros-pkg/robot_localization

# Upgrade protocol buffers gateway files
RUN cd /home/arcturus/dev_ws/src/protobuf_client_ros2/include/protobuf_client \
&& protoc --cpp_out=. gateway.proto

# Install remaining dependencies and build
WORKDIR /home/arcturus/dev_ws
RUN source /opt/ros/humble/setup.bash \
&& rosdep update \
&& rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
#&& colcon build --symlink-install

# Setup MOOS-IvP
WORKDIR /home/arcturus
RUN svn co https://oceanai.mit.edu/svn/moos-ivp-aro/trunk/ moos-ivp
RUN cd /home/arcturus/moos-ivp && ./build-moos.sh && ./build-ivp.sh
RUN git clone https://github.com/ArcturusNavigation/moos-ivp-arcturus
RUN cd /home/arcturus/moos-ivp-arcturus && ./build.sh

# Setup environment
WORKDIR /home/arcturus
ENV PATH=/home/arcturus/moos-ivp/bin:$PATH
ENV PATH=/home/arcturus/moos-ivp/scripts:$PATH
ENV PATH=/home/arcturus/moos-ivp-arcturus/bin:$PATH
ENV PATH=/home/arcturus/moos-ivp-arcturus/scripts:$PATH
ENV IVP_BEHAVIOR_DIRS=/home/arcturus/moos-ivp/lib
ENV IVP_IMAGE_DIRS=/home/arcturus/moos-ivp-arcturus/images

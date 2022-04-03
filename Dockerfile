# Development Docker image for Simulation of Jackal control w/ MPPI

FROM ros:noetic as ros-base

# install basic utilities
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
      vim \
      gdb \
      byobu \
      lsb-release \
      wget \
      gnupg \
    && rm -rf /var/apt/lists/*

# add ignition keys / PPA
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# install ignition
RUN apt-get update \
    && apt-get install -y \
      ignition-fortress \
    && rm -rf /var/apt/lists/*

# install ROS packages
RUN apt-get update \
    && apt-get install -y \
      python3-catkin-tools \
      ros-noetic-jackal-simulator \
      ros-noetic-jackal-desktop \
      ros-noetic-jackal-navigation \
    && rm -rf /var/apt/lists/*

# default drop into a byobu shell
CMD byobu
      

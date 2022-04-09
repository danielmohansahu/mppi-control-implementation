# Development Docker image for Simulation of Jackal control w/ MPPI

FROM ros:noetic as ros-base

# install basic utilities
RUN apt-get update \
    && apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      vim \
      gdb \
      byobu \
      lsb-release \
      wget \
      gnupg \
      python3-pip \
    && rm -rf /var/apt/lists/*

# add ignition keys / PPA
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# install ignition
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ignition-citadel \
      ros-noetic-ros-ign-gazebo \
      ros-noetic-ros-ign-bridge \
    && rm -rf /var/apt/lists/*

# install ROS packages
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      python3-catkin-tools \
      ros-noetic-jackal-description \
      ros-noetic-joint-state-publisher \
      ros-noetic-rviz \
    && rm -rf /var/apt/lists/*

# install needed python packages
RUN pip install --upgrade pip
RUN pip install -y \
      tqdm \
      matplotlib \
      pandas \
      seaborn \
      sklearn

# default drop into a byobu shell in expected mounting directory
WORKDIR /workspace
CMD byobu
      

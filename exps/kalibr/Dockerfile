FROM ros:melodic-perception-bionic
SHELL ["/bin/bash", "-c"]

# Settings
ENV ARCH=amd64
ENV DEBIAN_FRONTEND=noninteractive

# Permissions
RUN apt-get update && apt-get install -y sudo
ARG USERNAME=docker
ARG PASSWORD=docker
ARG UID=1000
ARG GID=1000
RUN useradd -m ${USERNAME} --uid=${UID} && echo "${USERNAME}:${PASSWORD}" | chpasswd
RUN adduser ${USERNAME} sudo
ENV HOME /home/$USERNAME

# Install basic things
USER root
RUN apt-get update -yq
RUN apt-get update && apt-get install -q -y \
  sudo \
  lsb-release \
  build-essential \
  git \
  cmake \
  vim \
  vifm \
  wget \
  libv4l-dev \
  libboost-dev \
  libceres-dev \
  libeigen3-dev \
  libeigen3-doc \
  libgtest-dev \
  libopencv-* \
  libyaml-cpp-dev \
  libglew-dev \
  python-catkin-tools \
  python-rosdep \
  python-igraph \
  python3-dev \
  python3-pip \
  python3-yaml \
  python3-numpy \
  python3-matplotlib \
  python3-matplotlib-dbg \
  python3-pandas \
  ros-melodic-cv-bridge \
  ros-melodic-image-transport \
  ros-melodic-tf \
  ros-melodic-tf-conversions \
  ros-melodic-rosbag

# Switch to $USERNAME
USER $USERNAME
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc
RUN echo 'source /opt/ros/melodic/setup.bash' >> $HOME/.bashrc

# Install Kalibr
WORKDIR $HOME
RUN mkdir -p $HOME/kalibr_ws/src \
  && cd $HOME/kalibr_ws/src \
  && git clone https://github.com/ethz-asl/kalibr \
  && cd .. \
  && source /opt/ros/melodic/setup.bash \
  && catkin build -DCMAKE_BUILD_TYPE=Release -j1

# Entry point
RUN echo '#!/bin/bash' >> $HOME/entry.bash
RUN echo 'source /opt/ros/melodic/setup.bash' >> $HOME/entry.bash
RUN echo 'exec "$@"' >> $HOME/entry.bash
RUN chmod +x $HOME/entry.bash
ENTRYPOINT ["/home/docker/entry.bash"]

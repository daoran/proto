FROM ros:kinetic-perception
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

# VINS-Fusion settings
ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/home/$USERNAME/catkin_ws

# Install base dependencies
USER root
RUN apt-get update -yq
RUN apt-get install -qq -y \
  cmake \
  libatlas-base-dev \
  libeigen3-dev \
  libgoogle-glog-dev \
  libsuitesparse-dev \
  python-catkin-tools \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-message-filters \
  ros-${ROS_DISTRO}-tf

# Install Ceres-Solver
WORKDIR $HOME
RUN git clone https://ceres-solver.googlesource.com/ceres-solver \
  && cd ceres-solver \
  && git checkout tags/${CERES_VERSION} \
  && mkdir build && cd build \
  && cmake .. \
  && make install

# Install VINS-Fusion
WORKDIR $CATKIN_WS
RUN mkdir -p src \
  && cd src \
  && git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
RUN catkin config \
    --extend /opt/ros/$ROS_DISTRO \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release
RUN catkin build

# Switch to $USERNAME
USER $USERNAME
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc
RUN echo 'source /opt/ros/kinetic/setup.bash' >> $HOME/.bashrc
RUN echo 'source $HOME/catkin_ws/devel/setup.bash' >> $HOME/.bashrc

# Entry point script
WORKDIR $HOME
RUN echo '#!/bin/bash' >> $HOME/entry.sh
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib' >> $HOME/entry.sh
RUN echo 'source /opt/ros/kinetic/setup.bash' >> $HOME/entry.sh
RUN echo 'source $HOME/catkin_ws/devel/setup.bash' >> $HOME/entry.sh
RUN echo 'exec "$@"' >> $HOME/entry.sh
RUN chmod +x $HOME/entry.sh
ENTRYPOINT ["/home/docker/entry.sh"]

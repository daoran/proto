FROM ros:kinetic-perception
SHELL ["/bin/bash", "-c"]

# Settings
ENV ARCH=amd64
ENV DEBIAN_FRONTEND=noninteractive

# Permissions
RUN apt-get update && apt-get install -y sudo
ARG USER=docker
ARG PASS=docker
ARG UID=1000
ARG GID=1000
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PASS}" | chpasswd
RUN adduser ${USER} sudo
ENV HOME /home/$USER

# Settings
ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=$HOME/catkin_ws

# Install base dependencies
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

# Install VINS-Mono
WORKDIR $CATKIN_WS
RUN mkdir -p src \
  && cd src \
  && git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
RUN catkin config \
    --extend /opt/ros/$ROS_DISTRO \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release
RUN catkin build

# Install tools
RUN apt-get install -qq -y \
  tmux \
  vim

# Switch to $USER
USER $USER
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc

# Entry point script
WORKDIR $HOME
COPY entry.sh /
ENTRYPOINT ["/entry.sh"]

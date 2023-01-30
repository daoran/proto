FROM ros:kinetic-perception
SHELL ["/bin/bash", "-c"]

# Permissions
ARG USER_ID
ARG GROUP_ID
ARG USERNAME="slam_bench"
RUN addgroup --gid $GROUP_ID $USERNAME
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID $USERNAME
ADD sudoers.txt /etc/sudoers
RUN chmod 440 /etc/sudoers
ENV HOME /home/$USERNAME

# Settings
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

# Switch to $USERNAME
USER $USERNAME
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc

# Entry point script
WORKDIR $HOME
COPY entry.sh /
ENTRYPOINT ["/entry.sh"]

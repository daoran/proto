FROM ros:melodic-perception-bionic
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

# Install basic things
USER root
RUN apt-get update -yq
RUN apt-get update && apt-get install -qq -y \
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
  ros-melodic-random-numbers \
  ros-melodic-pcl-conversions \
  ros-melodic-pcl-msgs \
  ros-melodic-pcl-ros \
  ros-melodic-cv-bridge \
  ros-melodic-image-transport \
  ros-melodic-message-filters \
  ros-melodic-tf \
  ros-melodic-tf-conversions \
  ros-melodic-rosbag

# Switch to $USERNAME
USER $USERNAME
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc

# Build msckf_vio
WORKDIR $HOME
RUN mkdir -p msckf_ws/src \
  && cd msckf_ws/src/ \
  && git clone https://github.com/KumarRobotics/msckf_vio \
  && cd .. \
  && source /opt/ros/melodic/setup.bash \
  && catkin build -j1

# Entry point script
WORKDIR $HOME
COPY entry.sh /
ENTRYPOINT ["/entry.sh"]

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

# Install Pangolin
WORKDIR $HOME
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
WORKDIR Pangolin
RUN mkdir -p build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE="Release" \
  && make -j1 && sudo make install

# Install ORBSLAM3
WORKDIR $HOME
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
WORKDIR ORB_SLAM3
RUN git checkout v1.0-release
COPY orbslam-no_loop_closure.patch .
RUN git apply orbslam-no_loop_closure.patch
RUN sed -i 's/find_package(OpenCV 4.4)/find_package(OpenCV)/g' CMakeLists.txt
# -- Build DBoW2
RUN cd Thirdparty/DBoW2 \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE="Release" \
  && make -j1 \
  && cd ../../../
# -- Build g2o
RUN cd Thirdparty/g2o \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE="Release" \
  && make -j1 \
  && cd ../../../
# -- Build Sophus
RUN cd Thirdparty/Sophus \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE="Release" \
  && make -j1 \
  && cd ../../../
# -- Uncompress vocabulary
RUN cd Vocabulary \
  && tar -xf ORBvoc.txt.tar.gz \
  && cd ..
# -- Build ORBSLAM3
RUN mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE="Release" \
  && make -j1 \
  && cd ../../
# -- Build ROS examples
RUN cd build \
  && cmake .. -DROS_BUILD_TYPE="Release" \
  && make -j1 \
  && cd ../../../../

# Switch to $USERNAME
USER $USERNAME
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc

# Entry point script
WORKDIR $HOME
COPY entry.sh /
ENTRYPOINT ["/entry.sh"]

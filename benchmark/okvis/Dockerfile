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
  libopencv-*

# Install OKVIS
WORKDIR $HOME
RUN git clone https://github.com/ethz-asl/okvis
WORKDIR okvis
RUN mkdir build \
  && cd build \
  && cmake -DCMAKE_BUILD_TYPE=Release .. \
  && make \
  && make install

# Switch to $USERNAME
USER $USERNAME
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc

# Entry point script
WORKDIR $HOME
COPY entry.sh /
ENTRYPOINT ["/entry.sh"]

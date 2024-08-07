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
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib' >>  $HOME/.bashrc

# Entry point script
WORKDIR $HOME
RUN echo "#!/bin/bash" >> $HOME/entry.sh
RUN echo 'exec "$@"' >> $HOME/entry.sh
RUN chmod +x $HOME/entry.sh
ENTRYPOINT ["/home/docker/entry.sh"]

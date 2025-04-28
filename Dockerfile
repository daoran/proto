FROM ubuntu:24.04

# Settings
ENV DEBIAN_FRONTEND=noninteractive
ARG USERNAME=docker
ARG PASSWORD=docker
ARG UID
ARG GID

# Remove ubuntu user
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu

# Create group and user with specified UID/GID
RUN apt-get update && apt-get install -y sudo adduser
RUN useradd -m ${USERNAME} --uid=${UID} && echo "${USERNAME}:${PASSWORD}" | chpasswd
RUN adduser ${USERNAME} sudo
ENV HOME /home/$USERNAME
ENV XYZ_DIR ${HOME}/xyz

# Build xyz dependencies
RUN mkdir -p ${XYZ_DIR}/third_party
COPY third_party/src/KHR ${XYZ_DIR}/third_party/src/KHR
COPY third_party/src/glad ${XYZ_DIR}/third_party/src/glad
COPY third_party/Makefile ${XYZ_DIR}/third_party/Makefile
COPY third_party/build_deps.bash ${XYZ_DIR}/third_party/build_deps.bash
RUN cd ${XYZ_DIR}/third_party && ./build_deps.bash

# Build xyz
# COPY . .
# RUN make libxyz

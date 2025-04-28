#!/bin/bash
set -e

# SETTINGS
BUILD_TYPE="Release"
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PREFIX=${SCRIPT_DIR}
NUM_CPU=$(nproc)
export CLICOLOR=0

# THIRD PARTY VERSIONS
# -- UTILS
YAMLCPP_TAG="0.8.0"
STB_TAG="master"
LZ4_TAG="v1.10.0"
GFLAGS_TAG="v2.2.2"
GLOG_TAG="v0.7.1"
# -- LINEAR ALGEBRA
SUITESPARSE_TAG="v7.10.2"
EIGEN_TAG="3.4.0"
CERES_TAG="2.2.0"
# -- COMPUTER VISION
OPENCV_TAG="4.10.0"
APRILTAG_IMGS_TAG="master"
APRILTAG_TAG="v3.4.3"
# -- COMPUTER GRAPHICS
GLFW_TAG="3.4"
ASSIMP_TAG="v5.4.3"

install_base() {
  sudo apt-get update -qqq

  # Base dev tools
  sudo apt-get install -y -qqq \
    build-essential \
    pkg-config \
    make \
    cmake \
    git \
    mercurial \
    g++ \
    clang \
    tcc

  # Computer graphics base
  sudo apt-get install -y -qqq \
    libx11-dev \
    libwayland-dev \
    libxkbcommon-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev

  # Linear algebra base
  sudo apt-get install -y -qqq \
    libomp-dev \
    libmpfr-dev \
    libblas-dev \
    liblapack-dev \
    liblapacke-dev \
    libmetis-dev
}

cmake_build() {
  # Go into repo
  cd "${PREFIX}/src/$1" || return

  # Prepare for build
  mkdir -p build
  cd build || return
  cmake \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DCMAKE_PREFIX_PATH="${PREFIX}" \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_SHARED_LIBS=1 \
    ..

  # Compile and install
  make -j${NUM_CPU} && make install
}

# $1 - Repo tag
# $2 - Git Repo URL
git_clone() {
  REPO_NAME=$(basename $2)
  cd "${PREFIX}/src" || exit
  if [ ! -d "${REPO_NAME}" ]; then
    git clone "$2" "${PREFIX}/src/${REPO_NAME}"
  fi
  cd ${REPO_NAME} && git checkout $1
  cd - > /dev/null || exit
}

# "$1" - Repo URL
# "$2" - Repo folder name
download_zip_repo() {
  # Download repo
  mkdir -p "${PREFIX}/src"
  cd "${PREFIX}/src" || return
  if [ ! -f "$2".zip ]; then
    wget --no-check-certificate "$1" -O "$2".zip
  fi
  unzip -oqq "$2".zip
  cd "$2" || return
}

###############################################################################
# UTILS
###############################################################################

install_base

# YAML-CPP
git_clone ${YAMLCPP_TAG} https://github.com/jbeder/yaml-cpp
cmake_build yaml-cpp

# STB
git_clone ${STB_TAG} https://github.com/nothings/stb
cd "${PREFIX}/include" \
  && ln -svf ${PREFIX}/src/stb/stb_c_lexer.h . \
  && ln -svf ${PREFIX}/src/stb/stb_connected_components.h . \
  && ln -svf ${PREFIX}/src/stb/stb_divide.h . \
  && ln -svf ${PREFIX}/src/stb/stb_ds.h . \
  && ln -svf ${PREFIX}/src/stb/stb_dxt.h . \
  && ln -svf ${PREFIX}/src/stb/stb_easy_font.h . \
  && ln -svf ${PREFIX}/src/stb/stb_herringbone_wang_tile.h . \
  && ln -svf ${PREFIX}/src/stb/stb_hexwave.h . \
  && ln -svf ${PREFIX}/src/stb/stb_image.h . \
  && ln -svf ${PREFIX}/src/stb/stb_image_resize.h . \
  && ln -svf ${PREFIX}/src/stb/stb_image_write.h . \
  && ln -svf ${PREFIX}/src/stb/stb_include.h . \
  && ln -svf ${PREFIX}/src/stb/stb_leakcheck.h . \
  && ln -svf ${PREFIX}/src/stb/stb_perlin.h . \
  && ln -svf ${PREFIX}/src/stb/stb_rect_pack.h . \
  && ln -svf ${PREFIX}/src/stb/stb_sprintf.h . \
  && ln -svf ${PREFIX}/src/stb/stb_textedit.h . \
  && ln -svf ${PREFIX}/src/stb/stb_tilemap_editor.h . \
  && ln -svf ${PREFIX}/src/stb/stb_truetype.h . \
  && ln -svf ${PREFIX}/src/stb/stb_voxel_render.h;

# LZ4
git_clone ${LZ4_TAG} https://github.com/lz4/lz4
cd ${PREFIX}/src/lz4 && make install PREFIX="${PREFIX}"

# GFLAGS
git_clone ${GFLAGS_TAG} https://github.com/gflags/gflags
cmake_build gflags

# GLOG
git_clone ${GLOG_TAG} https://github.com/google/glog
cmake_build glog

###############################################################################
# LINEAR ALGEBRA
###############################################################################

# SUITESPARSE
git_clone ${SUITESPARSE_TAG} https://github.com/DrTimothyAldenDavis/SuiteSparse
cmake_build SuiteSparse

# EIGEN
git_clone ${EIGEN_TAG} https://gitlab.com/libeigen/eigen.git
cmake_build eigen.git

# CERES
git_clone ${CERES_TAG} https://github.com/ceres-solver/ceres-solver
cmake_build ceres-solver

###############################################################################
# COMPUTER VISION
###############################################################################

# # OPENCV
# git_clone ${OPENCV_TAG} https://github.com/opencv/opencv
# cmake_build opencv

# APRILTAG
git_clone ${APRILTAG_IMGS_TAG} https://github.com/AprilRobotics/apriltag-imgs
git_clone ${APRILTAG_TAG} https://github.com/AprilRobotics/apriltag
cmake_build apriltag

###############################################################################
# COMPUTER GRAPHICS
###############################################################################

# KHR
cd "${PREFIX}/KHR" \
  && mkdir -p "${PREFIX}/include/KHR" \
  && ln -sf ${PREFIX}/KHR/khrplatform.h "${PREFIX}/include/KHR";

# GLAD
cd "${PREFIX}/glad" \
  && mkdir -p "${PREFIX}/include/glad" \
  && ln -sf ${PREFIX}/glad/glad.h "${PREFIX}/include/glad" \
  && gcc -I${PREFIX}/include -c glad.c -o glad.o \
  && ar rcs libglad.a glad.o \
  && rm glad.o \
  && mv ${PREFIX}/glad/libglad.a "${PREFIX}/lib/libglad.a";

# GLFW
git_clone ${GLFW_TAG} https://github.com/glfw/glfw
cmake_build glfw

# ASSIMP
git_clone ${ASSIMP_TAG} https://github.com/assimp/assimp
cmake_build assimp

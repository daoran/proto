#!/bin/bash
set -e  # Exit on first error
PREFIX=/usr/local/src/mexopencv
OPENCV_EXTRA_MODULES_PATH="${PREFIX}/opencv_contrib-3.4.0/modules"


# Install system dependencies
sudo apt-get install -q -y \
  octave liboctave-dev \
  build-essential cmake pkg-config git zlib1g-dev libjpeg-dev libpng-dev \
  libtiff-dev libopenexr-dev libavcodec-dev libavformat-dev libswscale-dev \
  libv4l-dev libdc1394-22-dev libxine2-dev libgphoto2-dev libgstreamer*-dev \
  libgstreamer-plugins-base*-dev libgtk*-dev libtbb-dev libeigen3-dev libblas-dev \
  liblapack-dev liblapacke-dev libatlas-base-dev

# Make the mexopencv environment
mkdir -p $PREFIX
cd $PREFIX

# Get OpenCV and OpenCV contrib
if [ ! -d opencv-3.4.0 ]; then
  wget -O opencv-3.4.0.zip https://github.com/opencv/opencv/archive/3.4.0.zip
fi
if [ ! -d opencv_contrib-3.4.0 ]; then
  wget -O opencv_contrib-3.4.0.zip https://github.com/opencv/opencv_contrib/archive/3.4.0.zip
fi
unzip -o opencv-3.4.0.zip
unzip -o opencv_contrib-3.4.0.zip

# Build and install OpenCV 3.4.0 to $PREFIX (defined above)
cd opencv-3.4.0
rm -rf build
mkdir -p build && cd build
cmake -G "Unix Makefiles" \
	-DBUILD_DOCS=OFF \
	-DBUILD_EXAMPLES=OFF \
	-DBUILD_PERF_TESTS=OFF \
	-DBUILD_TESTS=OFF \
	-DBUILD_JAVA=OFF \
	-DFORCE_VTK=OFF \
	-DWITH_CUDA=OFF \
	-DWITH_CUBLAS:BOOL=OFF \
	-DWITH_CUFFT:BOOL=OFF \
	-DWITH_NVCUVID:BOOL=OFF \
	-DWITH_MATLAB=OFF \
	-DBUILD_opencv_cudaarithm:BOOL=OFF \
	-DBUILD_opencv_cudabgsegm:BOOL=OFF \
	-DBUILD_opencv_cudacodec:BOOL=OFF \
	-DBUILD_opencv_cudafeatures2d:BOOL=OFF \
	-DBUILD_opencv_cudafilters:BOOL=OFF \
	-DBUILD_opencv_cudaimgproc:BOOL=OFF \
	-DBUILD_opencv_cudalegacy:BOOL=OFF \
	-DBUILD_opencv_cudaobjdetect:BOOL=OFF \
	-DBUILD_opencv_cudaoptflow:BOOL=OFF \
	-DBUILD_opencv_cudastereo:BOOL=OFF \
	-DBUILD_opencv_cudawarping:BOOL=OFF \
	-DBUILD_opencv_cudev:BOOL=OFF \
	-DBUILD_opencv_java=OFF \
	-DBUILD_opencv_js=OFF \
	-DBUILD_opencv_python2=OFF \
	-DBUILD_opencv_python3=OFF \
	-DBUILD_opencv_ts=OFF \
	-DBUILD_opencv_world=OFF \
	-DBUILD_opencv_matlab=OFF \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX=$PREFIX \
	-DOPENCV_ENABLE_NONFREE=ON \
	-DOPENCV_EXTRA_MODULES_PATH="$OPENCV_EXTRA_MODULES_PATH" \
  ..
make  # -j$(nproc)
make install
cd $PREFIX


# Get and install mexopencv
cd $PREFIX
if [ ! -d mexopencv ]; then
  git clone https://github.com/kyamagu/mexopencv
fi

cd mexopencv
export PKG_CONFIG_PATH=$PREFIX/lib/pkgconfig
export LD_LIBRARY_PATH=$PREFIX/lib
make WITH_OCTAVE=true WITH_CONTRIB=true all contrib
cd $PREFIX


# Create ~/.octaverc to auto load opencv
cat << EOF >> ~/.octaverc
warning('off', 'Octave:shadowed-function')
addpath('${PREFIX}/mexopencv')
addpath('${PREFIX}/mexopencv/opencv_contrib')
addpath('${PREFIX}/mexopencv/+cv/private')                 % HACK
addpath('${PREFIX}/mexopencv/opencv_contrib/+cv/private')  % HACK
EOF

# For MATLAB you do the following in the GUI
#
# cd('$PREFIX/mexopencv')
# addpath('$PREFIX/mexopencv')
# addpath('$PREFIX/mexopencv/opencv_contrib')
# cv.getBuildInformation()
#
# Where you replace '$PREFIX' with the setting at the top of this file

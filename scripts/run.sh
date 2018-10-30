set -e

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

# rm -rf build
mkdir -p build
cd build || return
cmake ..
time make -j8

cd tests
# ./calib-aprilgrid_test
./calib-calib_data_test
# ./core-config_test
# ./driver-camera-camera_test

# sudo make install

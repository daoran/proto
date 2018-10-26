# HEADERS=$(find include/ -type f)
# for HPP in $HEADERS; do
#   echo "Moving $HPP to ${HPP/include\/src}"
#   mv $HPP ${HPP/include\/src}
# done

# rm -rf build
mkdir -p build
cd build || return
cmake ..
time make -j8

cd tests
./core-config_test

# sudo make install

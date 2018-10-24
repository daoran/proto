#!/bin/bash
set -e  # halt on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

REPO_URL=https://svn.csail.mit.edu/apriltags
INC_DEST=${PREFIX}/include/
LIB_DEST=${PREFIX}/lib/libapriltags.a
REGEX_STRING='s/AprilTags\//apriltags\//g'
DOWNLOAD_PATH=${PREFIX}/src

install_dependencies()
{
    # Install dependencies
    apt-get install -qq -y \
        subversion \
        cmake \
        libeigen3-dev \
        libv4l-dev
}

install_apriltags()
{
    # Download and build mit apriltags
    cd "$DOWNLOAD_PATH"
    if [ ! -d apriltags ]; then
        svn --non-interactive --trust-server-cert co $REPO_URL
    fi
    cd apriltags

    # Use OpenCV 3 instead of 2
    sed -i \
      's/find_package(OpenCV)/find_package(OpenCV 3.2 REQUIRED)/g' \
      CMakeLists.txt

    # Make sure we can set TagFamily.blackBorder in TagDetector
    sed -i \
      's/const TagFamily thisTagFamily;/TagFamily thisTagFamily;/g' \
      AprilTags/TagDetector.h

    # Patch makefile so it builds with -fPIC
		CMAKE_PATCH=$(sed '4iset(CMAKE_POSITION_INDEPENDENT_CODE ON)' CMakeLists.txt)
		echo "$CMAKE_PATCH" | tee CMakeLists.txt

    # Download and build mit apriltags
    cd "$DOWNLOAD_PATH"
    if [ ! -d apriltags ]; then
        cat <<EOF > temp_file
#!/usr/bin/expect -f
spawn svn list ${REPO_URL}
expect "(R)eject, accept (t)emporarily or accept (p)ermanently?"
send -- "p\r"
svn --non-interactive --trust-server-cert --no-auth-cache co ${REPO_URL}
EOF
        chmod +x temp_file && ./temp_file && rm temp_file
        mv apriltags apriltags
    fi
    cd apriltags

    # Make
    make

    # Install
    # as of Aug 31st 2016 they don't have a install target
    # you have to install it manually
    mkdir -p $INC_DEST
    cp -r ./build/include/AprilTags $INC_DEST
    cp -r ./build/lib/libapriltags.a $LIB_DEST

    # # Do some hackery and change header file references
    # find $INC_DEST -type f -exec sed -i $REGEX_STRING {} +
}

uninstall_apriltags()
{
    rm -rf $INC_DEST
    rm $LIB_DEST
}


# RUN
install_dependencies
install_apriltags
# uninstall_apriltags

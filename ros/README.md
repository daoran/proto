# proto_ros

ROS wrapper for [`proto`](https://github.com/chutsu/proto).

## Build

For convenience there is a `Makefile` that automates the installation of
dependencies and building of `proto_ros`, the make targets are as follows.

    debug:
      Build in debug mode

    release:
      Build in release mode

Or, the standard way to build a catking project is to enter the following
commands:

    cd <your catkin_ws>/src
    git clone https://github.com/chutsu/proto_ros
    catkin build

## License

The source code is released under GPLv3 license.

# prototype

`prototype` contain notes and code used to learn about different aspects of
robotics. From estimation, mapping to control and planning.


## Build

For convenience there is a `Makefile` that automates the installation of
dependencies and building of `prototype`, the make targets are as follows.

    deps:
      Install prototype dependencies.

    debug:
      Build prototype in debug mode.

    release:
      Build prototype in release mode.

    install:
      Install prototype to $PREFIX. By default this is "/usr/local".

    format_code:
      Format prototype code using clang-format.

    docs:
      Generate docs for prototype.

The commands you usually care about are `debug`, `release` and `install`.

Or, if you're old-fashioned the standard way to build a C++ project is to enter
the following commands at the root of the repo.

    mkdir -p build
    cd build
    cmake ..
    make
    sudo make install  # By default will install to /usr/local

## License

The source code is released under GPLv3 license.

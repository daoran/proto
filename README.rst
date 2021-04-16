proto
=====

.. image:: https://github.com/chutsu/proto/workflows/ci/badge.svg
  :target: https://github.com/chutsu/proto/actions?query=ci

``proto`` contain notes and code used to learn about different aspects of
robotics. From estimation, mapping to control and planning.


Build
-----

For convenience there is a ``Makefile`` that automates the installation of
dependencies and building of ``proto``, the make targets are as follows::

    deps:
      Install proto dependencies.

    debug:
      Build proto in debug mode.

    release:
      Build proto in release mode.

    install:
      Install proto to $PREFIX. By default this is "/usr/local".

    ros:
      Build the ROS interface to `proto` (a.k.a `proto_ros`).

    format_code:
      Format proto code using clang-format.


License
-------

The source code is released under MIT license.

proto
=====

.. image:: https://github.com/chutsu/proto/workflows/ci/badge.svg
  :target: https://github.com/chutsu/proto/actions?query=ci

``proto`` contain notes and code used to learn about different aspects of
robotics. From estimation, mapping to control and planning.


Build
-----

For convenience there is a ``Makefile`` that automates the installation of
dependencies and building of ``proto``. To install dependencies, build and test
``proto`` simply issue the following commands:

.. code-block::

   make third_party
   make install
   make tests

alternatively just type ``make help`` to bring up other make targets.


License
-------

The source code is released under MIT license.

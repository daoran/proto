proto
=====

.. image:: https://github.com/chutsu/proto/actions/workflows/ci.yml/badge.svg
  :target: https://github.com/chutsu/proto/actions?query=ci

``proto`` contain notes and code used to learn about different aspects of
robotics.


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

.. code-block::

    Copyright (C) <2024>  <Christopher Choi>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.


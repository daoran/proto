Computer Vision Fundamentals
============================

Point on Line
-------------

.. math::

  \Transpose{\Vec{x}} \Vec{l} = 0



Intersection of Lines
---------------------

.. math::

  \begin{align}
    \Vec{l} (\Vec{l} \times \Vec{l}') \Vec{l}' (\Vec{l} \times \Vec{l}') &= 0 \\
    \Transpose{\Vec{l}} \Vec{x} = \Transpose{\Vec{l}}' \Vec{x} &= 0 \\
    \Vec{x} &= \Vec{l} \times \Vec{l}'
  \end{align}



Plane
-----

* A plane can be defined by the join between three points, or the join between
  a line and a point in general
* Two planes intersecting a unique line
* Three planes intersecting a unique point



Three Points Define a Plane
---------------------------

Suppose you have three points :math:`\Vec{p}_{1}`, :math:`\Vec{p}_{2}`,
:math:`\Vec{p}_{3}`, and are incident with a plane, :math:`\boldsymbol{\pi}`
then each point satisfies

.. math::

  \Transpose{\boldsymbol{\pi}} \Vec{p}_{i} = 0

By stacking each point as a matrix

.. math::

  \begin{bmatrix}
    \Transpose{\Vec{p}_{1}} \\
    \Transpose{\Vec{p}_{2}} \\
    \Transpose{\Vec{p}_{3}}
  \end{bmatrix} \boldsymbol{\pi} = 0

Since three points in general are rarely linearly independent, it follows that
the :math:`3 \times 4` matrix compsed of the points :math:`\Vec{p}_{i}` as rows
has rank 3.


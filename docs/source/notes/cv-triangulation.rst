Triangulation
=============

Linear Triangulation
--------------------

There are various methods for triangulating a 3D point obeserved from at least
two camera views. The linear triangulation method [Hartley2003] is frequently
used. This method assumes a pair of homogeneous points :math:`\Vec{x}` and
:math:`\Vec{x}' \in \real^{3}` in the image plane that observes the same 3D
point, :math:`\Vec{X} \in \real^{4}`, in homogeneous coordinates from two
different camera frames. The homogeneous projection from 3D to 2D with a known
camera matrix :math:`\Mat{P} \in \real^{3 \times 4}` for each measurement is
given as,

.. math::

  \begin{align}
    \Vec{x} &= \Mat{P} \Vec{X} \\
    \Vec{x}' &= \Mat{P}' \Vec{X}.
  \end{align}

Taking avantage of the fact if two vectors :math:`\Vec{x}` and
:math:`\Mat{P}\Vec{X}` have the same direction then :math:`\Vec{x} \times
\Mat{P} \Vec{X} = 0`.  These equations can be combined to form a system of
equations of the form :math:`\Mat{A} \Vec{x} = \Vec{0}`. To eliminate the
homogeneous scale factor we apply a cross product to give three equations for
each image point, for example :math:`\Vec{z} \times (\Mat{P} \Mat{X}) =
\Vec{0}` writing this out gives

.. math::

  x (\Vec{p}^{3T} \Vec{X}) - (\Vec{p}^{1T} \Vec{X}) = 0 \\
  y (\Vec{p}^{3T} \Vec{X}) - (\Vec{p}^{2T} \Vec{X}) = 0 \\
  x (\Vec{p}^{2T} \Vec{X}) - y (\Vec{p}^{1T} \Vec{X}) = 0

where :math:`\Vec{p}^{iT}` is the :math:`i^{\text{th}}` row of :math:`\Vec{P}`.
Note that the third line in the above equation is a linear combination of the
first two, (:math:`c_1` times first line plus :math:`c_2` times second line =
third line), as such the third line spans the space of the first two equations
and therefore is redundant.

From the above, an equation of the form :math:`\Mat{A} \Vec{x} = \Vec{0}` for
each image point can be formed, where :math:`\Vec{x}` represents the unknown
homogeneous feature location to be estimated, and :math:`\Mat{A}` is given as

.. math::

  \mathbf{A} =
  \begin{bmatrix}
    x (\Vec{p}^{3T}) - (\Vec{p}^{1T}) \\
    y (\Vec{p}^{3T}) - (\Vec{p}^{2T}) \\
    x' (\Vec{p'}^{3T}) - (\Vec{p'}^{1T}) \\
    y' (\Vec{p'}^{3T}) - (\Vec{p'}^{2T})
  \end{bmatrix}

giving a total of four equations in four homogeneous unknowns. Solving for
:math:`\Vec{A}` using SVD allows us to estimate the initial feature location.

In an ideal world, the position of 3D points can be solved as a system of
equations using the linear triangulation method. In reality, however, errors
are present in the camera poses and pixel measurements. The pixel measurements
observing the same 3D point are generally noisy. In addition, the camera models
and distortion models used often do not model the camera projection or
distortion observed perfectly. Therefore to obtain the best results an
iterative method should be used. This problem is generally formulated as a
non-linear least square problem and can be solved by numerical methods, such as
the Gauss-Newton algorithm.

[Hartley2003]: Hartley, Richard, and Andrew Zisserman. Multiple view geometry
in computer vision. Cambridge university press, 2003.

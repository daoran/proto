Pinhole Camera Model
====================

The pinhole camera model describes how 3D scene points are projected onto the
2D image plane of an ideal pinhole camera. The model makes the assumption that
light rays emitted from an object in the scene pass through the pinhole of the
camera, and projected onto the image plane.

A 3D point :math:`\Vec{p}_{C} = [p_x \quad p_y \quad p_z]^{\transpose}`
expressed in the camera frame, :math:`\frame_{C}`, projected on to the camera's
2D image plane :math:`(u, v)` is written as,

.. math::
  u = \dfrac{p_x f_{x}}{p_z} + c_x \quad v = \dfrac{p_y f_{y}}{p_z} + c_y

where :math:`f_{x}` and :math:`f_{y}` denote the focal lengths, :math:`c_{x}`
and :math:`c_{y}` represents the principal point offset in the :math:`x` and
:math:`y` direction. Or, in matrix form

.. math::

  \Vec{x}_{C} = \Mat{K} \Vec{p}_{C}

.. math::

  \begin{bmatrix}
  	u \\ v \\ 1
  \end{bmatrix} =
  \begin{bmatrix}
  	f_{x} & 0 & c_{x} \\
  	0 & f_{x} & c_{y} \\
  	0 & 0 & 1
  \end{bmatrix}
  \begin{bmatrix}
  	p_x / p_z \\ p_y / p_z \\ 1
  \end{bmatrix}

In practice, the pinhole camera model only serves as an approximation to modern
cameras. The assumptions made in the model are often violated with factors such
as large camera apertures (pinhole size), distortion effects in camera lenses,
and other factors. That is why the pinhole camera model is often used in
combination with a distortion model in the hope of minimizing projection errors
from 3D to 2D. Common distortion models used in conjuction with the pinhole
camera model includes:

* Radial-tangential distortion
* Equi-distant distortion



Guessing the focal length from the camera's field of view
---------------------------------------------------------

Using basic trigonometry, if we know the len's field of views we can obtain the
focal length in the :math:`x` and/or :math:`y` direction.

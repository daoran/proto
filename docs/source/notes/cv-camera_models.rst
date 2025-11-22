Camera Models
=============

Pinhole Camera Model
--------------------

.. _pinhole-camera-model:

The pinhole camera model describes how 3D scene points are projected onto the
2D image plane of an ideal pinhole camera. The model makes the assumption that
light rays emitted from an object in the scene pass through the pinhole of the
camera, and projected onto the image plane.

A 3D point :math:`\Vec{p}_{C} = [p_x \quad p_y \quad p_z]^{\transpose}`
expressed in the camera frame, :math:`\frame_{C}`, projected on to the camera's
2D image plane :math:`(u, v)` is written as,

.. math::

  u = \dfrac{p_{x} \cdot f_{x}}{p_{z}} + c_{x}
  \quad \quad
  v = \dfrac{p_{y} \cdot f_{y}}{p_{z}} + c_{y}

where :math:`f_{x}` and :math:`f_{y}` denote the focal lengths, :math:`c_{x}`
and :math:`c_{y}` represents the principal point offset in the :math:`x` and
:math:`y` direction. Or, in matrix form

.. math::

  \Vec{x}_{C} = \Mat{K} \cdot \Vec{p}_{C}

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

* :ref:`Radial-Tangential Distortion`
* :ref:`Equi-Distant Distortion`



Guessing the focal length from the camera's field of view
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using basic trigonometry, if we know the len's field of views we can obtain the
focal length in the :math:`x` and/or :math:`y` direction.




Radial-Tangential Distortion
----------------------------

.. _radtan-distortion:

Lens distortion generally exist in all camera lenses, therefore it is vital the
distortions observed are modelled. The most common distortion model is the
radial-tangential (or simply as radtan) distortion model. The two main
distortion components, as the name suggests, are the radial and tangential
distortion.

Radial distortion occurs due to the shape of the lens, where light passing
through the center undergoes no refraction, and light passing through the edges
of the lens, undergoes through severe bending causing the radial distortion.

.. figure:: imgs/cv-radial_distortion.png
  :align: center


Tangential distortion, on the other hand, is mainly due to camera sensor
mis-alignment during the manufacturing process. It occurs when the camera
sensor is not in parallel with the lens.

.. figure:: imgs/cv-tangential_distortion.png
  :align: center

The combined radial-tangential distortion is modelled using a polynomial
approximation with parameters :math:`k_{1}, k_{2}` and :math:`p_{1}, p_{2}`
respectively.  To apply the distortion the observed 3D point :math:`\Vec{p} =
[x \enspace y \enspace z]^{\transpose}` is first projected, distorted, and
finally scaled and offset in the image plane :math:`(u, v)`.

.. math::

  \begin{align}
    x &= X / Z \\
    y &= Y / Z \\
    r^2 &= x^2 + y^2 \\ \\
    x' &= x \cdot (1 + (k_1 r^2) + (k_2 r^4)) \\
    y' &= y \cdot (1 + (k_1 r^2) + (k_2 r^4)) \\
    x'' &= x' + (2 p_1 x y + p_2 (r^2 + 2 x^2)) \\
    y'' &= y' + (p_1 (r^2 + 2 y^2) + 2 p_2 x y)
  \end{align}


Radial Tangential Point Jacobian
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. math::

  \begin{align}
    \dfrac{\partial{\Vec{x}'}}{\partial{\Vec{x}}} &=
      \begin{bmatrix}
        J_{11} & J_{12} \\
        J_{21} & J_{22}
      \end{bmatrix} \\ \\
      r^2 &= x^2 + y^2 \\ \\
      J_{11} &= k_1 r^2 + k_2 r^4 + 2 p_1 y + 6 p_2 x
        + x (2 k_1 x + 4 k_2 x r^2) + 1 \\
      J_{12} &= 2 x p_1 + 2 y p_2 + y (2 k_1 x + 4 k_2 x r^2) \\
      J_{21} &= 2 x p_1 + 2 y p_2 + y (2 k_1 x + 4 k_2 x r^2) \\
      J_{22} &= k_1 r^2 + k_2 r^4 + 6 p_1 y + 2 p_2 x
        + y (2 k_1 y + 4 k_2 y r^2) + 1
  \end{align}


Radial Tangential Parameter Jacobian
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. math::

  \begin{align}
    \dfrac{\partial{\Vec{x}'}}{\partial{\Vec{d}}} &=
      \begin{bmatrix}
        J_{11} & J_{12} & J_{13} & J_{14} \\
        J_{21} & J_{22} & J_{23} & J_{24}
      \end{bmatrix} \\ \\
      r^2 &= x^2 + y^2 \\ \\
      J_{11} &= x r^2 \\
      J_{12} &= x r^4 \\
      J_{13} &= 2 x y \\
      J_{14} &= 3 x^2 + y^2 \\ \\
      J_{21} &= y r^2 \\
      J_{22} &= y r^4 \\
      J_{23} &= x^2 + 3 y^2 \\
      J_{24} &= 2 x y
  \end{align}



Equi-Distant Distortion
-----------------------

.. math::

  \begin{align}
    r &= \sqrt{x^{2} + y^{2}} \\
    \theta &= \arctan{(r)} \\
    \theta_d &= \theta (1 + k_1 \theta^2 + k_2 \theta^4
      + k_3 \theta^6 + k_4 \theta^8) \\
    x' &= (\theta_d / r) \cdot x \\
    y' &= (\theta_d / r) \cdot y
  \end{align}



Equi-distant Point Jacobian
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. math::

  \begin{align}
    \dfrac{\partial{\Vec{x}'}}{\partial{\Vec{x}}} &=
      \begin{bmatrix}
        J_{11} & J_{12} \\
        J_{21} & J_{22}
      \end{bmatrix} \\ \\
      %
      r &= \sqrt{x^{2} + y^{2}} \\
      \theta &= \arctan(r) \\
      \theta_d &= \theta (1 + k_1 \theta^2 + k_2 \theta^4
        + k_3 \theta^6 + k_4 \theta^8) \\
      \theta_d' &= 1 + 3 k_1 \theta^2 + 5 k_2 \theta^4
        + 7 k_3 \theta^6 + 9 k_4 \theta^8 \\
      \theta_r &= 1 / (r^2 + 1) \\ \\
      %
      s &= \theta_d / r \\
      s_r &= \theta_d' \theta_r / r - \theta_d / r^2 \\ \\
      %
      r_x &= 1 / r x \\
      r_y &= 1 / r y \\ \\
      %
      J_{11} &= s + x s_r r_x \\
      J_{12} &= x s_r r_y \\
      J_{21} &= y s_r r_x \\
      J_{22} &= s + y s_r r_y
  \end{align}



Equi-distant Parameter Jacobian
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. math::

  \begin{align}
    \dfrac{\partial{\Vec{x}'}}{\partial{\Vec{d}}} &=
      \begin{bmatrix}
        J_{11} & J_{12} & J_{13} & J_{14} \\
        J_{21} & J_{22} & J_{23} & J_{24}
      \end{bmatrix} \\ \\
    r &= \sqrt{x^{2} + y^{2}} \\
    \theta &= \arctan(r) \\ \\
    J_{11} &= x \theta^3 / r \\
    J_{12} &= x \theta^5 / r \\
    J_{13} &= x \theta^7 / r \\
    J_{14} &= x \theta^9 / r \\ \\
    J_{21} &= y \theta^3 / r \\
    J_{22} &= y \theta^5 / r \\
    J_{23} &= y \theta^7 / r \\
    J_{24} &= y \theta^9 / r
  \end{align}

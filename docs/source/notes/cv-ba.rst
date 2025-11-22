Bundle Adjustment
=================

.. math::

  \Argmin{\Tf{W}{C}, \Pt{W}{P}}
    \Norm{\Vec{z} - \boldsymbol{\pi}(\Tf{W}{C}^{-1} \enspace \Pt{W}{P})}^{2} \\
  % -- Project chain
  \boldsymbol{\pi} =
    \boldsymbol{k}(
    \boldsymbol{d}(
    \boldsymbol{p}(
      \Tf{W}{C}^{-1} \enspace \Pt{W}{P}
    )))


Useful skew properties:

.. math::

  \begin{align}
    \Skew{\Vec{v}}^{\transpose} &= -\Skew{\Vec{v}} \\
    \Skew{\Vec{v}}^{2}
      &= -\Vec{v}\Vec{v}^{\transpose}
        - \Vec{v}^{\transpose} \Vec{v} \I \\
    \Skew{\rot({\Phi}) \Vec{v}}
    &= \rot({\Phi}) \Skew{\Vec{v}} \rot({\Phi})^{\transpose}
  \end{align}



Derivative w.r.t Project
------------------------

.. math::

  \begin{align}
  % -- Project
  \Vec{x}
    &= \boldsymbol{p}(\Tf{W}{C}^{-1} \enspace \Pt{W}{P}) \\
    &= \boldsymbol{p}(\Pt{C}{P}) \\
    &= \begin{bmatrix}
      x / z \\
      y / z \\
    \end{bmatrix}
  \end{align}


.. math::

  \begin{align}
  \dfrac{\partial{\Vec{x}}}{\partial{\Pt{C}{P}}}
    &=
      \begin{bmatrix}
        1 / z & 0 & -x / z^{2} \\
        0 & 1 / z & -y / z^{2}
      \end{bmatrix}
  \end{align}



Derivative w.r.t Radial-Tangential Distortion
---------------------------------------------

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


.. math::

  \begin{align}
  \dfrac{\partial{\Vec{x}'}}{\partial{\Vec{x}}} &=
    \begin{bmatrix}
      J_{11} & J_{12} \\
      J_{21} & J_{22}
    \end{bmatrix} \\ \\
    J_{11} &= k_1 r^2 + k_2 r^4 + 2 p_1 y + 6 p_2 x
      + x (2 k_1 x + 4 k_2 x r^2) + 1 \\
    J_{12} &= 2 x p_1 + 2 y p_2 + y (2 k_1 x + 4 k_2 x r^2) \\
    J_{21} &= 2 x p_1 + 2 y p_2 + y (2 k_1 x + 4 k_2 x r^2) \\
    J_{22} &= k_1 r^2 + k_2 r^4 + 6 p_1 y + 2 p_2 x
      + y (2 k_1 y + 4 k_2 y r^2) + 1
  \end{align}


.. math::

  \begin{align}
  \dfrac{\partial{\Vec{x}'}}{\partial{\Vec{d}_{\text{params}}}} &=
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



Derivative w.r.t. Scale and Center
----------------------------------

.. math::

  u = f_x \cdot x' + c_x \\
  v = f_y \cdot y' + c_y


.. math::

  \dfrac{\partial\hat{\Vec{z}}}{\partial\Vec{x}'} &= \begin{bmatrix}
    f_x & 0 \\
    0 & f_y
  \end{bmatrix}



Derivative w.r.t Camera Pose :math:`\Tf{W}{C}`
----------------------------------------------

.. math::

  \begin{align}
  \Pt{C}{P} &= \Tf{W}{C}^{-1} \enspace \Pt{W}{P} \\
    &= \Rot{W}{C}^{-1} \enspace \Pt{W}{P} - \Rot{W}{C}^{-1} \Trans{W}{C}
  \end{align}


.. math::

  \begin{align}
   \dfrac{\partial\hat{\Vec{z}}}{\partial\Vec{x}'}
   \dfrac{\partial\Vec{x}'}{\partial\Vec{x}}
   \dfrac{\partial\Vec{x}}{\partial\Pt{C}{P}}
   \dfrac{\partial{\Pt{C}{P}}}{\partial{\Tf{W}{C}}}
  \end{align}


.. math::

  \begin{align}
    \dfrac{\partial{\Pt{C}{P}}}{\partial{\Tf{W}{C}}}
      &= \begin{bmatrix}
        \dfrac{\partial{\Pt{C}{P}}}{\partial{\quat_{WC}}}
    \enspace
        \dfrac{\partial{\Pt{C}{P}}}{\partial{\Trans{W}{C}}}
      \end{bmatrix}
  \end{align}


.. math::

  \begin{align}
    \dfrac{\partial{\Pt{C}{P}}}{\partial{\quat_{WC}^{-1}}}
      &= -\Skew{\Rot{W}{C}^{-1} \left( \Pt{W}{P} - \Trans{W}{C} \right)} \\
    \dfrac{\partial{\quat_{WC}^{-1}}}{\partial{\quat_{WC}}}
      &= -\Rot{W}{C}^{-1} \\ \\
    \dfrac{\partial{\Pt{C}{P}}}{\partial{\quat_{WC}^{-1}}}
    \dfrac{\partial{\quat_{WC}^{-1}}}{\partial{\quat_{WC}}}
      &= (-\Skew{\Rot{W}{C}^{-1} \left( \Pt{W}{P} - \Trans{W}{C} \right)})
         (-\Rot{W}{C}^{-1}) \\
      & \text{using skew property:} \enspace \Skew{\rot \enspace \Vec{v}}
        = \rot \Skew{\Vec{v}} \rot^{\transpose} \\
      &= (-\Rot{W}{C}^{-1} \Skew{\left( \Pt{W}{P} - \Trans{W}{C} \right)}
        \enspace \Rot{W}{C})(-\Rot{W}{C}^{-1}) \\
      &= \Rot{W}{C}^{-1} \Skew{\left( \Pt{W}{P} - \Trans{W}{C} \right)} \\
      \\ \\
    \dfrac{\partial{\Pt{C}{P}}}{\partial{\Trans{W}{C}}}
      &= -\Rot{W}{C}^{-1}
  \end{align}



Derivative w.r.t Landmark :math:`\Pt{W}{P}`
-------------------------------------------

.. math::

  \Pt{C}{P} &= \Tf{W}{C}^{-1} \enspace \Pt{W}{P} \\
    &= \Rot{W}{C}^{-1} \enspace \Pt{W}{P} - \Rot{W}{C}^{-1} \Trans{W}{C}


.. math::

   \dfrac{\partial\hat{\Vec{z}}}{\partial\Vec{x}'}
      \dfrac{\partial\Vec{x}'}{\partial\Vec{x}}
      \dfrac{\partial\Vec{x}}{\partial\Pt{C}{P}}
      \dfrac{\partial{\Pt{C}{P}}}{\partial{\Pt{W}{P}}}


.. math::

  \dfrac{\partial\Pt{C}{P}}{\partial\Pt{W}{P}} = \Rot{W}{C}^{-1}

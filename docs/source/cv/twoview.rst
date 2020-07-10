Two-View Geometry
=================


Essential Matrix
----------------

.. math::

   \Vec{x} \Mat{E} \Vec{x}' = 0


Scalar Tripple Product
^^^^^^^^^^^^^^^^^^^^^^

.. math::

   \begin{align}
     \Vec{a} \times \Vec{b}
     =&\enspace (a_y b_z - a_z b_y) i \\
     &+ (a_z b_x - a_x b_x) j \\
     &+ (a_x b_y - a_y b_x) k
   \end{align}


.. math::

  \begin{align}
    (\Vec{a} \times \Vec{b}) \cdot \Vec{c}
    =&\enspace (a_y b_z - a_z b_y) c_x \\
    &+ (a_z b_x - a_x b_x) c_y \\
    &+ (a_x b_y - a_y b_x) c_z \\
    =& \underbrace{| \Vec{a} \times \Vec{b} |}_{\text{Area of Parallelogram}}
       \cdot
       \underbrace{| \Vec{c} | \cos{\theta}}_{\text{Height of Parallelepiped}}
  \end{align}


Derivation of the Essential Matrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. math::

  \begin{align}
    \Vec{p}_{1} &= \lambda_{1} \Vec{x}_{1} \\
    \Vec{p}_{2} &= \lambda_{2} \Vec{x}_{2} \\
    \Vec{p}_{2} &= \rot \Vec{p}_{1} + \pos
  \end{align}


.. math::

  \begin{align}
    % Line 1
      \Vec{p}_{2} &= \rot \Vec{p}_{1} + \pos \\
    % Line 2
      \lambda_{1} \Vec{x}_{1}
        &= \rot \lambda_{2} \Vec{x}_{2} + \pos \\
    % Line 3
      \lambda_{1} \Skew{\pos} \Vec{x}_{1}
        &= \Skew{\pos} \rot \lambda_{2} \Vec{x}_{2}
        + \underbrace{\Skew{\pos} \pos}_{=0} \\
    % Line 4
      \lambda_{1}
      \underbrace{\Vec{x}_{1}^{\transpose} \Skew{\pos} \Vec{x}_{1}}_{= 0}
        &= \Vec{x}_{1}^{\transpose} \Skew{\pos} \rot \lambda_{2} \Vec{x}_{2} \\
    % Line 5
      \Vec{x}_{1}^{\transpose}
        \underbrace{\Skew{\pos} \rot}_{\Mat{E}}
        \Vec{x}_{2}
        &= 0
  \end{align}


.. math::

  \begin{align}
    \boxed{
      \Vec{x}_{1}^{\transpose} \Mat{E} \; \Vec{x}_{2} = 0
    }
  \end{align}


Fundamental Matrix
------------------

.. math::

  \Vec{x}^{\transpose} \Mat{F} \; \Vec{x}' = 0


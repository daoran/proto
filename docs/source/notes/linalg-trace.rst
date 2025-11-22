Trace
=====

.. math::

  \text{tr}(\Mat{A}) = \sum_{i} \Mat{A}_{ii}

The trace of a matrix :math:`\Mat{A}` is simply the sum of all of its diagonal.
For example, let :math:`\Mat{A}` be a matrix, with

.. math::
  \Mat{A} = \begin{bmatrix}
    a_{11} & a_{12} & a_{13} \\
    a_{21} & a_{22} & a_{23} \\
    a_{31} & a_{32} & a_{33}
  \end{bmatrix} =
  \Mat{A} = \begin{bmatrix}
    -1 & 0 & 3 \\
    11 & 5 & 2 \\
    6 & 12 & -5
  \end{bmatrix}

then

.. math::

  \Trace{\Mat{A}}
    = \sum_{i = 1}^{3} a_{ii}
    = a_{11} + a_{22} + a_{33}
    = -1 + 5 + (-5)
    = -1


Trace Properties
----------------

.. math::

  % Property 1
  \Trace{\Mat{A} + \Mat{B}} &= \Trace{\Mat{A}} + \Trace{\Mat{B}} \\
  % Property 2
  \Trace{c \Mat{A}} &= c \; \Trace{\Mat{A}} \\
  % Property 3
  \Trace{\Mat{A}} &= \Trace{\Mat{A}}^{\transpose} \\
  % Property 4
  \Trace{\Mat{A}^{\transpose} \Mat{B}}
    &= \Trace{\Mat{A} \Mat{B}^{\transpose}}
    = \Trace{\Mat{B}^{\transpose} \Mat{A}}
    = \Trace{\Mat{B} \Mat{A}^{\transpose}} \\
  % Property 5
  \Trace{\Mat{A} \Mat{B}} &= \Trace{\Mat{B} \Mat{A}} \\
  % Property 6
  \Trace{\Vec{b} \Vec{a}^{\transpose}} &= \Vec{a}^{\transpose} \Vec{b} \\
  % Property 7
  \Trace{\Mat{ABCD}} &=
    \Trace{\Mat{BCDA}} =
    \Trace{\Mat{CDAB}} =
    \Trace{\Mat{DABC}} & \text{Cyclic Property}\\
  % Property 8
  \Trace{\Mat{ABC}} &\neq \Trace{\Mat{ACB}} & \text{iff not symmetric} \\
  % Property 9
  \Trace{\Mat{AB}} &\neq \Trace{\Mat{A}} \; \Trace{\Mat{B}} \\
  % Property 10
  \Trace{\Mat{A} \otimes \Mat{B}} &= \Trace{\Mat{A}} \; \Trace{\Mat{B}}
    & \text{Trace of Kronecker product} \\


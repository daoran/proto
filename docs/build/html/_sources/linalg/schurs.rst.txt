Shurs' Complement
=================

Let :math:`\Mat{M}` be a matrix that consists of block matrices
:math:`\Mat{A}`, :math:`\Mat{B}`, :math:`\Mat{C}`, :math:`\Mat{D}`,

.. math::

  \Mat{M} =
  \begin{bmatrix}
    \Mat{A} & \Mat{B} \\
    \Mat{C} & \Mat{D}
  \end{bmatrix}

if :math:`\Mat{A}` is invertible, the Schur's complement of the block
:math:`\Mat{A}` of the matrix :math:`\Mat{B}` is the defined by

.. math::

  \begin{aligned}
    \Mat{M}/\Mat{A} &= \Mat{D} - \Mat{C} \Mat{A}^{-1} \Mat{B} \\
    \Mat{M}/\Mat{D} &= \Mat{A} - \Mat{B} \Mat{D}^{-1} \Mat{C}
  \end{aligned}

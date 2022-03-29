Linear Least Squares
====================

Linear problems generally have the form

.. math::

  \Mat{A} \Vec{x} = \Vec{b}

If :math:`\Mat{A}` is skinny (number of rows is larger than number of columns)
the problem is over constrained and there is no *unique* solution.
Instead, the problem can be solved by minizming the squared error between
:math:`\Mat{A} \Vec{x}` and :math:`\Vec{b}`. The linear least squares problem
is then defined as,

.. math::

  \min_{\Vec{x}} || \Mat{A} \Vec{x} - \Vec{b} ||^{2}_{2}

where the goal is to find an *approximate* solution.

The local minima can be found when the derivative of the squared error is
zero. First the squared error is expanded to give:

.. math::
  (\Mat{A} \Vec{x} - \Vec{b})^{\transpose}
    (\Mat{A} \Vec{x} - \Vec{b}) \\
  \Transpose{\Vec{x}} \Transpose{\Mat{A}} \Mat{A} \Vec{x}
    - 2 \Transpose{\Vec{b}} \Mat{A} \Vec{x}

then by differentiating the expanded squared error with respect to
:math:`\Vec{x}`, setting the derivative to zero, and rearranging the equation
with respect to :math:`\Vec{x}` gives the following:

.. math::

  % Line 1
  2 \Transpose{\Vec{x}} \Transpose{\Mat{A}} \Mat{A}
    - 2 \Transpose{\Vec{b}} \Mat{A} &= 0 \\
  % Line 2
  \Transpose{\Vec{x}} \Transpose{\Mat{A}} \Mat{A}
    &= \Transpose{\Vec{b}} \Mat{A} \\
  % Line 3
  \Transpose{\Mat{A}} \Mat{A} \Vec{x}
    &= \Transpose{\Mat{A}} \Vec{b} \\
  % Line 4
  \Vec{x}
    &= \left( \Transpose{\Mat{A}} \Mat{A} \right)^{-1}
      \Transpose{\Mat{A}} \Vec{b} \\
  % Line 5
  \Vec{x}
    &= \Mat{A}^{\dagger} \Vec{b} \enspace,

where :math:`\left( \Transpose{\Mat{A}} \Mat{A} \right)^{-1}
\Transpose{\Mat{A}}` is known as the pseudo inverse :math:`\Mat{A}^{\dagger}`.

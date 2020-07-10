Condition Number
================

There are different condition numbers. In the following the condition number
for the problem :math:`\Mat{A} \Vec{x} = \Vec{b}` and matrix inversion are
discussed.  In general, the condition number, :math:`\kappa(\cdot)`, for a
matrix, :math:`\Mat{A}`, or computational task such as :math:`\Mat{A} \Vec{x} =
\Vec{b}` measures how sensitive the output is to perturbations in the input
data and to round off errors. If the condition number is large, even a small
error in :math:`\Vec{x}` would cause a large error in :math:`\Vec{x}`. On the
other hand, if the condition number is small then the error in :math:`\Vec{x}`
will not be much bigger than the error in :math:`\Vec{b}`.

.. math::

  \kappa(\Mat{A}) &\approx 1 \quad \text{well-Conditioned} \\
  \kappa(\Mat{A}) &> 1 \quad \text{ill-Conditioned}

The condition number is defined more precisely to be the maximum ratio of the
relative error in :math:`\Vec{x}` to the relative error in :math:`\Vec{b}`.

Let :math:`\Vec{e}` be the error in :math:`\Vec{b}`. Assuming that
:math:`\Mat{A}` is a nonsingular matrix, the error in the solution
:math:`\Mat{A}^{-1} \Vec{b}` is :math:`\Mat{A}^{-1} \Vec{e}`. The ratio of the
relative error in the solution to the relative error in :math:`\Vec{b}` is

.. math::

   \dfrac{
      \dfrac{\Norm{\Mat{A}^{-1} \Vec{e}}}{\Norm{\Mat{A}^{-1} \Vec{b}}}
   }{
      \dfrac{\Norm{\Vec{e}}}{\Norm{\Vec{b}}}
   }

which can be rewritten as,

.. math::

  \left(
    \dfrac{\Norm{\Mat{A}^{-1} \Vec{e}}}{\Norm{\Vec{e}}}
  \right)
  \cdot
  \left(
    \dfrac{\Norm{\Vec{b}}}{\Norm{\Mat{A}^{-1} \Vec{b}}}
  \right)

The maximum value (for nonzero :math:`\Vec{b}` and :math:`\Vec{e}`) is then
seen to be the product of the two operator norms as follows:

.. math::

  % -- LINE 1
  &\max_{\Vec{e}, \Vec{b} \neq 0}
  \left\{
    \left(
      \dfrac{\Norm{\Mat{A}^{-1} \Vec{e}}}{\Norm{\Vec{e}}}
      \cdot
      \dfrac{\Norm{\Vec{b}}}{\Norm{\Mat{A}^{-1} \Vec{b}}}
    \right)
  \right\} \\
  % -- LINE 2
  &= \max_{\Vec{e}, \Vec{b} \neq 0}
  \left\{
    \left(
      \dfrac{\Norm{\Mat{A}^{-1} \Vec{e}}}{\Norm{\Vec{e}}}
    \right)
  \right\}
  \cdot
  \max_{\Vec{e}, \Vec{b} \neq 0}
  \left\{
    \left(
      \dfrac{\Norm{\Vec{b}}}{\Norm{\Mat{A}^{-1} \Vec{b}}}
    \right)
  \right\} \\
  % -- LINE 3
  &= \max_{\Vec{e}, \Vec{b} \neq 0}
  \left\{
    \left(
      \dfrac{\Norm{\Mat{A}^{-1} \Vec{e}}}{\Norm{\Vec{e}}}
    \right)
  \right\}
  \cdot
  \max_{\Vec{e}, \Vec{b} \neq 0}
  \left\{
    \left(
      \dfrac{\Norm{\Mat{A} \Vec{x}}}{\Norm{\Vec{x}}}
    \right)
  \right\} \\
  % -- LINE 4
  &= \Norm{\Mat{A}^{-1}} \cdot \Norm{\Mat{A}}

The same definition is used for any matrix norm, i.e. one that satisfies

.. math::

  \kappa(\Mat{A}) = \Norm{\Mat{A}^{-1}} \cdot \Norm{\Mat{A}}
    \geq \Norm{\Mat{A}^{-1} \cdot \Mat{A}} = 1 .

When the condition number is exactly one (which can only happen if
:math:`\Mat{A}` is a scalar multiple of a linear isometry), then a solution
algorithm can find (in principle, meaning if the algorithm introduces no errors
of its own) an approximation of the solution whose precision is no worse than
that of the data.

However, it does not mean that the algorithm will converge rapidly to this
solution, just that it won't diverge arbitrarily because of inaccuracy on the
source data (backward error), provided that the forward error introduced by the
algorithm does not diverge as well because of accumulating intermediate
rounding errors.

The condition number may also be infinite, but this implies that the problem is
ill-posed (does not possess a unique, well-defined solution for each choice of
data -- that is, the matrix is not invertible), and no algorithm can be
expected to reliably find a solution.

The definition of the condition number depends on the choice of norm, as can be
illustrated by two examples.

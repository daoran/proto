Marginalization
===============

As a reminder, marginalization is about having a joint density `p(x, y)`
over two variables `x` and `y`, and we would like to marginalize out or
"eliminate a variable", lets say `y` in this case:

.. math::
  p(x) = \int_{y} p(x, y)

resulting in a density `p(x)` over the remaining variable `x`.

Now, if the density was in covariance form with mean :math:`\boldsymbol{\mu}`
and covariance :math:`\mathbf{\Sigma}`, partitioned as follows:

.. math::
  p(x, y) = \mathcal{N}(
    % Mean
    \begin{bmatrix}
      \boldsymbol\mu_{x} \\ 
      \boldsymbol\mu_{y}
    \end{bmatrix},
    % Covariance
    \begin{bmatrix}
      \mathbf\Sigma_{xx}, \mathbf\Sigma_{xy} \\ 
      \mathbf\Sigma_{yx}, \mathbf\Sigma_{yy}
    \end{bmatrix}
  )

marginalization is simple, as the corresponding sub-block
:math:`\mathbf{\Sigma}_{xx}` already contains the covariance on `x` i.e.,

.. math::
  p(x) = \mathcal{N}(
    % Mean
    \boldsymbol\mu_{x},
    % Covariance
      \mathbf\Sigma_{xx}
  ).

In the nonlinear-least squares formulation, however, we can only obtain the
covariance :math:`\mathbf{\Sigma}` with the following property,

.. math::
  \mathbf{\Sigma} = \Mat{H}^{-1}


where :math:`\Mat{H}` is the Hessian matrix in a Gauss-Newton system
(:math:`\Mat{H}\delta\Vec{x} = \Vec{b}`).



Using Shur's Complement for marginalization
-------------------------------------------

First let :math:`\state_1` be the states to be marginalized out,
:math:`\state_{2}` be the set of states related to those by error terms, and
:math:`\state_{\rho}` be the set of remaining states. Partitioning the Hessian, error
state and R.H.S of the Gauss-Newton system gives:

.. math::
  \begin{bmatrix}
    \Mat{H}_{11} & \Mat{H}_{12} \\
    \Mat{H}_{21} & \Mat{H}_{22}
  \end{bmatrix}
  \begin{bmatrix}
    \delta\state_{1} \\
    \delta\state_{2}
  \end{bmatrix}
  =
  \begin{bmatrix}
    \Vec{b}_{1} \\
    \Vec{b}_{2}
  \end{bmatrix}

and applying the Shur complement operation yields:

.. math::
  \Mat{H}^{\ast}_{22}
  &=
  \Mat{H}_{22} -
  \Mat{H}_{21}
  \Mat{H}_{11}^{-1}
  \Mat{H}_{12}
  \\
  \Vec{b}^{\ast}_{2}
  &=
  \Vec{b}_{2} -
  \Mat{H}_{21}
  \Mat{H}_{11}^{-1}
  \Vec{b}_{1}

where :math:`\Vec{b}^{\ast}_{2}` and
:math:`\Mat{H}^{\ast}_{22}` are non-linear functions of
:math:`\state_2` and :math:`\state_1`.


Derivation of Schur's Complement for Marginalization
----------------------------------------------------


From the Gauss-Newton system, :math:`\Mat{H} \delta\Vec{x} = \Vec{b}`, we can
derive the marginalization of the old states in :math:`\delta\Vec{x}` algebraically.
Let us decompose the system as:

.. math::
  % H
  \begin{bmatrix}
  \Mat{H}_{11} & \Mat{H}_{12} \\ 
  \Mat{H}_{21} & \Mat{H}_{22}
  \end{bmatrix}
  % x
  \begin{bmatrix}
  \delta\Vec{x}_{1} \\ 
  \delta\Vec{x}_{2}
  \end{bmatrix}
  =
  % b
  \begin{bmatrix}
  \Vec{b}_{1} \\ 
  \Vec{b}_{2}
  \end{bmatrix}

If we multiply out the block matrices and vectors out we get:

.. math::
  % Line 1
  \Mat{H}_{11} \delta\Vec{x}_{1} + \Mat{H}_{12} \delta\Vec{x}_{2}
  = \Vec{b}_{1} \\
  % Line 2
  \Mat{H}_{21} \delta\Vec{x}_{1} + \Mat{H}_{22} \delta\Vec{x}_{2}
  = \Vec{b}_{2}

Suppose we want to marginalize out the :math:`\delta\Vec{x}_{2}`, the second equation
above can be rearranged w.r.t. :math:`\delta\Vec{x}_{2}` like so:

.. math::
  % Line 1
  \Mat{H}_{21} \delta\Vec{x}_{1} + \Mat{H}_{22} \delta\Vec{x}_{2}
  &= \Vec{b}_{2} \\
  % Line 2
  \Mat{H}_{22} \delta\Vec{x}_{2}
  &= \Vec{b}_{2} - \Mat{H}_{21} \delta\Vec{x}_{1} \\
  % Line 3
  \delta\Vec{x}_{2}
  &= \Mat{H}_{22}^{-1} \Vec{b}_{2}
  - \Mat{H}_{22}^{-1} \Mat{H}_{21} \delta\Vec{x}_{1} \\

substituting :math:`\delta\Vec{x}_{2}` back into :math:`\Mat{H}_{11} \delta\Vec{x}_{1} +
\Mat{H}_{12} \delta\Vec{x}_{2} = \Vec{b}_{1}`, and rearranging the terms so it is
w.r.t :math:`\delta\Vec{x}_{1}` to get:

.. math::
  % Line 1
  \Mat{H}_{11} \delta\Vec{x}_{1} + \Mat{H}_{12}
  (\Mat{H}_{22}^{-1} \Vec{b}_{2}
  - \Mat{H}_{22}^{-1} \Mat{H}_{21} \delta\Vec{x}_{1})
  &= \Vec{b}_{1} \\
  % Line 2
  \Mat{H}_{11} \delta\Vec{x}_{1}
  + \Mat{H}_{12} \Mat{H}_{22}^{-1} \Vec{b}_{2}
  - \Mat{H}_{12} \Mat{H}_{22}^{-1} \Mat{H}_{21} \delta\Vec{x}_{1}
  &= \Vec{b}_{1} \\
  % Line 3
  (\Mat{H}_{11}
  - \Mat{H}_{12}\Mat{H}_{22}^{-1}\Mat{H}_{21}) \delta\Vec{x}_{1}
  &= \Vec{b}_{1} - \Mat{H}_{12} \Mat{H}_{22}^{-1} \Vec{b}_{2}

thus the Schur Complement of :math:`\Mat{H}_{22}` in :math:`\Mat{H}` is:

.. math::
  \Mat{H} / \Mat{H}_{22} &=
    \Mat{H}_{11}
    - \Mat{H}_{12} \Mat{H}_{22}^{-1} \Mat{H}_{21} \\
    \Vec{b} / \Vec{b}_{2} &=
      \Vec{b}_{1} - \Mat{H}_{12} \Mat{H}_{22}^{-1} \Vec{b}_{2}

If you want to marginalize out :math:`\delta\Vec{x}_{1}` you can follow the same
process above but w.r.t :math:`\Vec{x}_{1}`.



First Estimate Jacobians (FEJ)
------------------------------

In the context of real time state-estimation, a fixed-lag smoother provides a
way to bound the optimization problem in order to operate in real time. The
method to remove old states in the state vector is called *marginalization*. To
perform marginalization the Schur's complement is used to marginalize out old
states.

Simply performing marginalization, however, introduces estimator
inconsistencies.

Let us consider the following scenario. A state vector, :math:`\state`, during
the time interval :math:`[0, k]` will contain `m` old states to be marginalized
out and `r` remain states which we wish to keep. i.e. :math:`\state =
[\state_{m}^{\transpose} \quad \state_{r}^{\transpose}]^{\transpose}`. Then the
cost function, `c(\cdot)`, can be written as a function of :math:`\state` at
time `k` as,

.. math::
  c(\state_{k}) &= c(\state_{m}, \state_{r}) \\
                &= c(\state_{m}) + c(\state_{r}).

The intuition behind the above is since the state at time `k` can be
partitioned into `m` and `r`, the cost can also be decomposed. Utilizing this
property, the multivariate optimization can also be decomposed as follows,

.. math::

  \min_{\state_{m}, \state_{r}} c(\state_{m}, \state_{r})
    &= \min_{\state_{r}} (\min_{\state_{m}} c(\state_{m}, \state_{r})) \\
    &= \min_{\state_{r}} (c(\state_{r}) + \min_{\state_{m}} c(\state_{m})) .


The equation above shows the minimization problem can be solved by first
optimizing for the states :math:`\state_{m}`, and then forming a prior towards
the problem of solving for :math:`\state_{r}`. The reformulation of the
minimization problem entails no approximation.

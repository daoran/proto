Gauge Freedom
=============

Gauge theory is borrowed from physics.

Accurate structure from motion or vision based state estimation is hard. One
hurdle is addressing the accuracy quantitatively. There are two main problems
that arise:

* **Inherent Physical Indeterminancy**: cause by loss of information
  while projecting 3D objects onto a 2D image plane.
* **Overparameterized Problem**: e.g. a shape model that can be
  parameterized by a vector, each representing the absolute position and
  orientation of the object could itself be indeterminant.

It is well known that a vision only bundle adjustment has 7 unobserable
degrees-of-freedom (DoF), while for a VI-system, the global position and global
yaw is not observable, a total of four unobservable DoFs. These unobservable
DoFs (a.k.a gauge freedoms) have to be handled properly.

There are three main approaches to address the unobservability in a VI-system.
They are:

* Gauge fixation
* Gauge prior
* Free gauge



Gauge fixation
--------------

Gauge fixation method works by decreasing the number of optimization parameters
to where there are no unobservable states left for the opitmization problem to
optimize. This is to ensure the Hessian is well conditioned and invertable.
This approach enforces hard constraints to the solution.

The standard method to update orientation variables such as a rotation,
:math:`\rot`, during the iterations of a non-linear least squares solver is to
use local coordinates, where at the `k`-th iteration, the update is

.. math::
  :label: opt-rot_std_update

  \rot^{k + 1} = \text{Exp}(\delta \boldsymbol{\phi}^{k}) \rot^{k} .

Setting the :math:`z` component of :math:`\boldsymbol{\phi}^{k}` to 0 allows
fixating the yaw with respect to :math:`\rot^{k}`. However, concatenating
several such updates over :math:`K`-iterations,

.. math::
  \rot^{K} = \prod^{K-1}_{k=0} \text{Exp}(\delta \boldsymbol{\phi}^{k}) ,

does not fixate the yaw with respect to the initial rotation :math:`\rot^{0}`,
and therefore, this parameterization cannot be used to fix the yaw-value of
:math:`\rot^{K}` to that of the initial value :math:`\rot^{0}`.

Although pose fixation or prior can be applied to any camera pose, it is common
practice to fixate the first camera.

.. math::

  \pos_{0} = \pos^{0}_{0} ,
  \enspace
  \Delta \boldsymbol{\phi}_{0 z}
    \dot{=} \, \Vec{e}^{\transpose}_{z} \boldsymbol{\phi}_{0} = 0 \, ,


where :math:`\pos^{0}_{0}` is the initial position of the first camera. Which
is equivalent to setting the corresponding columns of the Jacobian of the
residual vector to zero, namely :math:`\jac_{\pos_0} = 0`, :math:`\jac_{\Delta
\phi_{0 z}} = 0`.  Thus, for rotations of the other camera poses, the standard
iterative update Eq.~\eqref{eq:opt-rot_std_update} is used, and, for the first
camera rotation, :math:`\rot_{0}`, a more convenient parameterization is used.
Instead of directly using :math:`\rot_{0}`, a left-multiplicative increment is
used.

.. math::

  \rot_{0} = \text{Exp}(\Delta \boldsymbol{\phi}_{0}) \rot^{0}_{0} \, ,

where the rotation vector :math:`\Delta \boldsymbol{\phi}_{0}` is initialized
to zero and updated.


Gauge prior
-----------

Gauge prior augments the objective function with an additional penalty to favor
a solution that satisfies certain constraints in a soft manner.

.. math::

  \Norm{\Vec{e}^{\pos}_{0}}^{2}_{\Sigma^{\pos}_{0}} \, ,
  \quad \text{where} \quad
  \Vec{e}^{\pos}_{0}(\boldsymbol{\theta})
    \enspace \dot{=} \enspace
    (\pos_{0} - \pos^{0}_{0}, \enspace \Delta \phi_{0 z})



Free gauge
----------

Free gauge is the most general, lets the optimization parameters evolve freely.
In order to deal with the singularity with the Hessian, the pseudo inverse is
used or some preconditioning method inorder to make the Hessian
well-conditioned and invertible.

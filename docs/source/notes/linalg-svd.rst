SVD Decomposition
=================

.. math::
  \Mat{A} = \Mat{U} \Mat{\Sigma} \Mat{V}^{\ast}


How to compute SVD
------------------

* Compute eigenvalues and eigenvectors of :math:`\Mat{A}^{\transpose} \Mat{A}`

   .. math::

      \begin{align}
         \Mat{A}^{\transpose} \Mat{A} \Vec{v}_{1} &= \lambda_{1} \Vec{v}_{1} \\
         &\vdots \\
         \Mat{A}^{\transpose} \Mat{A} \Vec{v}_{n} &= \lambda_{n} \Vec{v}_{n} \\
      \end{align}


* Make matrix :math:`\Mat{V}` from the vectors :math:`\Vec{v}_{i}`

    .. math::

      \Mat{V} =
      \begin{bmatrix}
        \vert &  & \vert \\
        \Vec{v}_{1} & \dots  & \Vec{v}_{n} \\
        \vert &  & \vert \\
      \end{bmatrix}


* Make a diagonal matrix :math:`\Mat{\Sigma}` from the square roots of the eigen
  values.

    .. math::

      \Mat{\Sigma} =
      \begin{bmatrix}
        \sqrt{\lambda_1} & 0 & 0 & 0 & 0 \\
        0 & \sqrt{\lambda_2} & 0 & 0 & 0 \\
        0 & 0 & \ddots & 0 & 0 \\
        0 & 0 & 0 & \ddots & 0 \\
        0 & 0 & 0 & 0 & \sqrt{\lambda_{n}}
      \end{bmatrix}


* Find :math:`\Mat{U}` from

   .. math::

     \Mat{A} = \Mat{U} \Mat{\Sigma} \Mat{V}^{\transpose}

   .. math::

     \Mat{U} \Mat{\Sigma} = \Mat{A} \Mat{V}

  In the simplest case:

    .. math::

      \Mat{U} = \Mat{A} \Mat{V} \Mat{\Sigma}^{-1}

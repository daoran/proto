Inverting a Matrix
==================

Inverting a matrix in linear algebra is often avoidable and recommended.

* Inverting a Matrix with Cholesky Decomposition
* Pseudo Inverse of a Matrix with SVD



Inverting a Matrix with Cholesky Decomposition
----------------------------------------------

Let matrix :math:`\Mat{A}` be invertible, using the identity:

.. math::

  \Mat{A} \Mat{A}^{-1} = \I

we can solve for the inverse of :math:`\Mat{A}` by using Cholesky
decomposition, but first lets rewrite the above by first decomposing
:math:`\Mat{A} = \Mat{L} \Mat{L}^{\transpose}` and rearrange the terms such
that we can take advantage of back substition,

.. math::

  \begin{align}
    (\Mat{L} \Mat{L}^{\transpose}) \Mat{A}^{-1} &= \I \\
    (\Mat{L}^{\transpose}) \Mat{L} \Mat{A}^{-1} &= \I \\
    \Mat{L} \Mat{A}^{-1} &= (\Mat{L}^{\transpose})^{-1}  \\
  \end{align}


Pseudo Inverse of a Matrix with SVD
-----------------------------------

.. math::

  \Mat{A} = \Mat{U} \Mat{\Sigma} \Mat{V}^{\transpose}


.. math::

  \begin{align}
    \Mat{A}^{\dagger}
       &= (\Mat{A}^{\transpose} \Mat{A})^{-1} \Mat{A}^{\transpose} \\
       &= ((\Mat{U} \Mat{\Sigma} \Mat{V}^{\transpose})^{\transpose}
             (\Mat{U} \Mat{\Sigma} \Mat{V}^{\transpose}))^{-1}
          (\Mat{U} \Mat{\Sigma} \Mat{V}^{\transpose})^{\transpose} \\
       &= ((\Mat{V} \Mat{\Sigma} \Mat{U}^{\transpose}
             \Mat{U} \Mat{\Sigma} \Mat{V}^{\transpose})^{-1}
          (\Mat{U} \Mat{\Sigma} \Mat{V}^{\transpose})^{\transpose} \\
       &= (\Mat{V} \Mat{\Sigma}^{2} \Mat{V}^{\transpose})^{-1}
             \Mat{V} \Mat{\Sigma} \Mat{U}^{\transpose} \\
       &= (\Mat{V}^{\transpose})^{-1} \Mat{\Sigma}^{-2} \Mat{V}^{-1}
             \Mat{V} \Mat{\Sigma} \Mat{U}^{\transpose} \\
       &= \Mat{V} \Mat{\Sigma}^{-2} \Mat{\Sigma} \Mat{U}^{\transpose} \\
       &= \Mat{V} \Mat{\Sigma}^{-1} \Mat{U}^{\transpose} \\
  \end{align}

where :math:`\Mat{\Sigma}^{-1}` is the pseudo inverse of :math:`\Mat{\Sigma}`,
which is formed by replacing everey non-zero diagonal entry by its reciprocal
and transposing the resulting matrix.


Invert Lower Triangular Matrix
------------------------------

If we have a lower triangular matrix :math:`\Mat{L}` and our objective is to
find :math:`\Mat{L}^{-1}`, we can use the property

.. math::

  \begin{align}
    \Mat{L}\Mat{L}^{-1} = \I,
  \end{align}

and use forward-substitution to solve for :math:`\Mat{L}^{-1}` column by
column, staring from the first column :math:`j`.

.. math::

  \begin{align}
    \begin{bmatrix}
      l_{11} & 0 & \dots & 0 \\
      l_{21} & l_{22} & \dots & 0 \\
      \vdots & \vdots & \ddots & \vdots \\
      l_{m1} & l_{m2} & \dots & l_{mn}
    \end{bmatrix}
    \begin{bmatrix}
      a_{1j} \\
      a_{2j} \\
      \vdots \\
      a_{mj}
    \end{bmatrix}
    =
    \begin{bmatrix}
      1 \\
      0 \\
      \vdots \\
      0
    \end{bmatrix}
  \end{align}

.. math::

  \begin{align}
    & l_{11} a_{1j} = 1 \\
    & l_{21} a_{1j} + l_{22} a_{2j} = 0 \\
    & \qquad\qquad\qquad\vdots \\
    & l_{m1} a_{1j} + l_{m2} a_{2j} + \dots + l_{mn} a_{mj} = 0
  \end{align}

.. math::

  \begin{align}
    & a_{1j} = 1 / l_{11} \\
    & a_{2j} = - l_{21} a_{1j} / l_{22} \\
    & \qquad\qquad\qquad\vdots \\
    & a_{mj} = (-l_{m1} a_{1j} - l_{m2} a_{2j}
                - \dots - l_{m,j-1} a_{m-1,j}) / l_{mn}
  \end{align}

.. math::

  \begin{align}
    a_{ij} = \dfrac{I_{ij} - \sum_{j=i+1}^{1} l_{ij} a_{i}}{l_{ii}}
    \quad
    \text{where} \; i = n, n - 1, \cdots, 1
  \end{align}

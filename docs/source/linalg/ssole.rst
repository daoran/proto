Solving System of Linear Equations
==================================

.. math::

  \Mat{A} \Vec{x} = \Vec{b}



Forward Substitution
--------------------

.. math::

  \Mat{L} \Vec{x} = \Vec{b}


.. math::

  \begin{bmatrix}
    l_{11} & 0 & \dots & 0 \\
    l_{21} & l_{22} & \dots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    l_{m1} & l_{m2} & \dots & l_{mn}
  \end{bmatrix}
  \begin{bmatrix}
    x_{1} \\
    x_{2} \\
    \vdots \\
    x_{m}
  \end{bmatrix}
  =
  \begin{bmatrix}
    b_{1} \\
    b_{2} \\
    \vdots \\
    b_{m}
  \end{bmatrix}

writing out the above,

.. math::
  &l_{11} x_{1} = b_{1} \\
  &l_{21} x_{1} + l_{22} x_{2} = b_{2} \\
  &l_{31} x_{1} + l_{32} x_{2} + l_{33} x_{3} = b_{3} \\
  &\qquad\qquad\qquad\vdots \\
  &l_{m,1} x_{1} + l_{m,2} x_{2} + \dots + l_{m,n} x_{n} = b_{n}

and rearranging to solve for :math:`\Vec{x}`,

.. math::

  x_{1} &= b_{1} / l_{11} \\
  x_{2} &= (b_{2} - l_{21} x_{1}) / l_{22} \\
  x_{3} &= (b_{3} - l_{31} x_{1} - l_{32} x_{2} ) / l_{33} \\
  &\qquad\qquad\qquad\qquad\qquad\vdots \\
  x_{m} &= (b_{m} - l_{m,1} x_{1} - l_{m,2} x_{2} - \dots
            - l_{m,m-1} x_{m-1} ) / l_{m,n}

or more generally,

.. math::
  \boxed{
    x_{i} = \dfrac{b_{i} - \sum_{j=1}^{i-1} l_{ij} x_{i}}{l_{ii}}
    \quad
    \text{where} \; 1 \leq i \leq n
  }.



Backward Substitution
---------------------

.. math::

  \Mat{U} \Vec{x} = \Vec{b}


.. math::

  \begin{bmatrix}
    u_{11} & u_{12} & \dots & u_{1n} \\
    0 & u_{22} & \dots & u_{2n} \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \dots & u_{mn} \\
  \end{bmatrix}
  \begin{bmatrix}
    x_{1} \\
    x_{2} \\
    \vdots \\
    x_{m}
  \end{bmatrix}
  =
  \begin{bmatrix}
    b_{1} \\
    b_{2} \\
    \vdots \\
    b_{m}
  \end{bmatrix}

writing out the above,

.. math::

  \begin{align}
    &u_{11} x_{1} + u_{12} x_{2} + \dots + u_{1n} x_{n} = b_{1} \\
    &u_{22} x_{2} + \dots + u_{2n} x_{n} = b_{2} \\
    &\qquad\qquad\qquad\vdots \\
    &u_{mn} x_{n} = b_{n}
  \end{align}

and rearranging to solve for :math:`\Vec{x}`,

.. math::

  \begin{align}
    x_{1} &= (b_{1} - u_{12} x_{2} - \dots - u_{1n} x_{n}) / u_{11} \\
    x_{2} &= (b_{2} - u_{22} x_{3} - \dots - u_{2n} x_{n}) / u_{22} \\
    &\qquad\qquad\qquad\vdots \\
    x_{m} &= b_{m} / u_{mn}
  \end{align}

or more generally,

.. math::

  \boxed{
    x_{i} = \dfrac{b_{i} - \sum_{j=i+1}^{1} u_{ij} x_{i}}{u_{ii}}
    \quad
    \text{where} \; i = n, n - 1, \cdots, 1
  }.



Solve Least Squares with SVD
----------------------------

To solve :math:`\Mat{A} \Vec{x} = \Vec{b}` with non-singular :math:`\Mat{A} \in
\real^{n \times n}`, lets factor :math:`\Mat{A}` using SVD and rearrange the
terms gives,

.. math::

  \Mat{A} \Vec{x} &= \Vec{b} \\
  \Mat{U} \Mat{\Sigma} \Mat{V}^{\transpose} \Vec{x} &= \Vec{b} \\
  \Mat{\Sigma} \Mat{V}^{\transpose} \Vec{x} &= \Mat{U}^{\transpose} \Vec{b}

Note: :math:`\Mat{U}` and :math:`\Mat{V}` are orthogonal matrices, therefore
the inverse is its transpose. Let :math:`\Vec{y} = \Vec{V}^{\transpose}
\Vec{x}` and subbing into the above gives,

.. math::
   \Mat{\Sigma} \Vec{y} = \Mat{U}^{\transpose} \Vec{b},

solve :math:`\Vec{y}` via forward substitution. Once :math:`\Vec{y}` is known
solve for :math:`\Vec{x}` in,

.. math::
  \Mat{V}^{\transpose} \Vec{x} = \Vec{y}

using back-substitution.



Solve Least Squares with QR
---------------------------

To solve :math:`\Mat{A} \Vec{x} = \Vec{b}` with non-singular :math:`\Mat{A} \in
\real^{n \times n}`, lets factor :math:`\Mat{A}` using QR decomposition and
rearrange the terms gives,

.. math::

  \Mat{A} \Vec{x} &= \Vec{b} \\
  \Mat{Q} \Mat{R} \Vec{x} &= \Vec{b} \\
  \Mat{R} \Vec{x} &= \Mat{Q}^{\transpose} \Vec{b}.

Note: :math:`\Mat{Q}` is an orthogonal matrix, therefore the inverse of
:math:`\Mat{Q}` is its transpose. The R.H.S. of the last equation is simply
matrix products of :math:`\Mat{Q}^{\transpose}`,  and :math:`\Vec{b}` which are
known. Once the R.H.S is computed, :math:`\Vec{x}` can be solved using
back-substitution.



Solve Least Squares with Cholesky Decomposition
-----------------------------------------------

To solve :math:`\Mat{A} \Vec{x} = \Vec{b}` with non-singular :math:`\Mat{A} \in
\real^{n \times n}`, lets factor :math:`\Mat{A}` using Cholesky decomposition
gives,

.. math::

  \Mat{A} \Vec{x} &= \Vec{b} \\
  \Mat{L} \Mat{L}^{\transpose} \Vec{x} &= \Vec{b},

let :math:`\Vec{y} = \Mat{L}^{\transpose} \Vec{x}`, subbing into the above,

.. math::

  \Mat{L} \Vec{y} = \Vec{b}.

Solve for :math:`\Vec{y}` using forward-substitution, and then solve for
:math:`\Vec{x}` in

.. math::

  \Mat{L}^{\transpose} \Vec{x} = \Vec{y}

using backward-substitution.

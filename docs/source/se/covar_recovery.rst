Covariance Recovery
===================

The Hessian matrix :math:`\Mat{H}` is known to be related to the marginal
covariance matrix :math:`\covar` by :math:`\covar = \Mat{H}^{-1}`. However,
inverting :math:`\Mat{H}` can be expensive and impossible if it is not
well-conditioned. The objective in the following is to recover the marginal
covariance matrix without explicitly inverting :math:`\Mat{H}`.

This can be done by decomposing the Hessian :math:`\Mat{H}` is into a lower and
upper triangular matrix, :math:`\Mat{R}^{\transpose} \Mat{R}` using either
Cholesky or QR factorization. Then let us write,

.. math::
  (\Mat{R}^{\transpose} \Mat{R})
  (\Mat{R}^{\transpose} \Mat{R})^{-1}
    &= \I \\
  (\Mat{R}^{\transpose} \Mat{R}) \covar &= \I \\
  \Mat{R} \covar &= (\Mat{R}^{\transpose})^{-1}.

and using back-substitution on the last equation to solve for :math:`\covar`
results in the following two general equations for any diagonal and
off-diagonal values of :math:`\covar`:

.. math::
  \boxed{
    \sigma_{ii} =
    \dfrac{1}{u_{ii}}
    \left(
      l_{ii}
      -\sum_{j=i+1}^{n} u_{i,j} \sigma_{j,i}
    \right)
  }


.. math::
  \boxed{
    \sigma_{il} =
    \dfrac{1}{u_{ii}}
    \left(
      -\sum_{j=i+1}^{l} u_{i,j} \sigma_{j,l}
      -\sum_{j=l+1}^{n} u_{i,j} \sigma_{j,l}
    \right)
  }

**Note** that the summations only apply to non-zero entries of single
columns or rows of the sparse matrix :math:`\Mat{R}`.



Derivation of Covariance Recovery using Square Root Matrix
----------------------------------------------------------

.. math::

  \Mat{R} \covar = (\Mat{R}^{\transpose})^{-1}


.. math::

  \underbrace{
    \begin{bmatrix}
      u_{11} & u_{12} & \cdots & u_{1n} \\
      0 & u_{22} & \cdots & u_{2n} \\
      \vdots & \vdots & \ddots & \vdots \\
      0 & 0 & \cdots & u_{nn} \\
    \end{bmatrix}
  }_{\Mat{R}}
  \underbrace{
    \begin{bmatrix}
      \sigma_{1,1} & \sigma_{1,2} & \dots & \sigma_{1,n} \\
      \sigma_{2,1} & \sigma_{2,2} & \dots & \sigma_{2,n} \\
      \vdots & \vdots & \vdots & \vdots \\
      \sigma_{n,1} & \sigma_{n,2} & \dots & \sigma_{n,n} \\
    \end{bmatrix}
  }_{\covar}
  =
  \underbrace{
  \begin{bmatrix}
    l_{11} & 0 & \dots & 0 \\
    l_{21} & l_{22} & \dots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    l_{m1} & l_{m2} & \dots & l_{nn}
  \end{bmatrix}
  }_{(\Mat{R}^{\transpose})^{-1}}


The trick to solving for :math:`\covar` is the fact the term
:math:`(\Mat{R}^{\transpose})^{-1}` is not actually evaluated. Instead, we take
advantage of the struture of the lower triangular matrix to solve a system of
equations via back-substituion. For one, we know for a fact the inverse of the
diagonals in :math:`(\Mat{R}^{\transpose})^{-1}` is the reciprocal of itself,
i.e.  :math:`l_{ii} = 1 / u_{ii}` (a known property of inverting diagonal
matrices).  Secondly, by performing back-substition the lower triangle values
of :math:`(\Mat{R}^{\transpose})^{-1}` are not required.

Lets see an example, suppose :math:`\Mat{R}`, :math:`\covar`, and
:math:`(\Mat{R}^{\transpose})^{-1}` are :math:`4 \times 4` matrices,

.. math::

  \begin{bmatrix}
    u_{11} & u_{12} & u_{13} & u_{14} \\
    0 & u_{22} & u_{23} & u_{24} \\
    0 & 0 & u_{33} & u_{34} \\
    0 & 0 & 0 & u_{44}
  \end{bmatrix}
  \begin{bmatrix}
    \sigma_{11} & \sigma_{12} & \sigma_{13} & \sigma_{14} \\
    \sigma_{21} & \sigma_{22} & \sigma_{23} & \sigma_{24} \\
    \sigma_{31} & \sigma_{32} & \sigma_{33} & \sigma_{34} \\
    \sigma_{41} & \sigma_{42} & \sigma_{43} & \sigma_{44}
  \end{bmatrix}
  =
  \begin{bmatrix}
    l_{11} & 0 & 0 & 0 \\
    l_{21} & l_{22} & 0 & 0 \\
    l_{31} & l_{32} & l_{33} & 0 \\
    l_{41} & l_{42} & l_{43} & l_{44}
  \end{bmatrix},

to workout :math:`\covar` we only need to find the values of the diagonals and
upper triangular matrix of :math:`\covar` (because a covariance matrix is
symmetrical).  If we write out the matrix multiplication, and rearrange w.r.t
values of :math:`\covar` for each column in :math:`\covar` we get:


**1st Column of** :math:`\covar`

.. math::

   \begin{align}
      u_{11} \sigma_{11}
         + u_{12} \sigma_{21}
         + u_{13} \sigma_{31}
         + u_{14} \sigma_{41} &= l_{11}
   \end{align}

.. math::

   \begin{align}
     \sigma_{11} &=
       (l_{11} -u_{12} \sigma_{21}
       - u_{13} \sigma_{31}
       - u_{14} \sigma_{41}) / u_{11}
   \end{align}


**2nd Column of** :math:`\covar`

.. math::
    u_{11} \sigma_{12}
      + u_{12} \sigma_{22}
      + u_{13} \sigma_{32}
      + u_{14} \sigma_{42} &= 0 \\
    u_{22} \sigma_{22}
      + u_{23} \sigma_{32}
      + u_{24} \sigma_{42} &= l_{22}


.. math::
    \sigma_{12} &= (-u_{12} \sigma_{22}
      - u_{13} \sigma_{32} - u_{14} \sigma_{42}) / u_{11} \\
    \sigma_{22} &= (l_{22} -u_{23} \sigma_{32}
      - u_{24} \sigma_{42}) / u_{22}


**3rd Column of** :math:`\covar`

.. math::
    u_{11} \sigma_{13}
      + u_{12} \sigma_{23}
      + u_{13} \sigma_{33}
      + u_{14} \sigma_{43} &= 0 \\
    u_{22} \sigma_{23}
      + u_{23} \sigma_{33}
      + u_{24} \sigma_{43} &= 0 \\
    u_{33} \sigma_{33}
      + u_{34} \sigma_{43} &= l_{33}


.. math::
    \sigma_{13} &= (-u_{12} \sigma_{23}
      - u_{13} \sigma_{33}
      - u_{14} \sigma_{43}) / u_{11} \\
    \sigma_{23} &= (-u_{23} \sigma_{33}
      - u_{24} \sigma_{43}) / u_{22} \\
    \sigma_{33} &= (l_{33} - u_{34} \sigma_{43}) / u_{33}


**4th Column of** :math:`\covar`

.. math::

   u_{11} \sigma_{14}
      + u_{12} \sigma_{24}
      + u_{13} \sigma_{34}
      + u_{14} \sigma_{44} &= 0 \\
   u_{22} \sigma_{24} + u_{23} \sigma_{34} + u_{24} \sigma_{44} &= 0 \\
   u_{33} \sigma_{34} + u_{34} \sigma_{44} &= 0 \\
   u_{44} \sigma_{44} &= l_{44}

.. math::

   \sigma_{14} &= (-u_{12} \sigma_{24}
      - u_{13} \sigma_{34}
      - u_{14} \sigma_{44}) / u_{11} \\
   \sigma_{24} &= (-u_{23} \sigma_{34}
      - u_{24} \sigma_{44}) / u_{22}  \\
   \sigma_{34} &= (-u_{34} \sigma_{44}) / u_{33} \\
   \sigma_{44} &= l_{44} / u_{44}


Collecting the diagonal and off-diagonal terms we can form general equations to
find any values in :math:`\covar`:

**Diagonals**

.. math::
   \begin{align}
      % Line 1
      \color{blue}{\sigma_{11}} &=
        (\color{brown}{l_{11}}
        \color{magenta}{-u_{12} \sigma_{21}
        - u_{13} \sigma_{31}
        - u_{14} \sigma_{41}})
        / \color{red}{u_{11}} \\
      % Line 2
      \color{blue}{\sigma_{22}} &=
        (\color{brown}{l_{22}}
        \color{magenta}{-u_{23} \sigma_{32}
        - u_{24} \sigma_{42}})
        / \color{red}{u_{22}} \\
      % Line 3
      \color{blue}{\sigma_{33}} &=
        (\color{brown}{l_{33}}
        \color{magenta}{-u_{34} \sigma_{43}})
        / \color{red}{u_{33}} \\
      % Line 4
      \color{blue}{\sigma_{44}} &=
         \color{brown}{l_{44}} / \color{red}{u_{44}}
   \end{align}

.. math::

  \begin{align}
    \color{blue}{{\sigma}_{ii}} =
      \color{red}{\dfrac{1}{{u}_{ii}}}
      \left(
        \color{brown}{l_{ii}}
        \color{magenta}{-{\sum}_{j=i+1}^{n} u_{i,j} {\sigma}_{j,i}}
      \right)
  \end{align}

Since we know that the inverse of the diagonals are its reciprocal, `l_{ii}`
can be written as :math:`\frac{1}{u_{ii}}` giving us the general formula for the
diagonals of :math:`\covar` as,

.. math::

  \boxed{
  \color{blue}{\sigma_{ii}} =
    \color{red}{\dfrac{1}{u_{ii}}}
    \left(
      \color{brown}{\dfrac{1}{u_{ii}}}
      \color{magenta}{-\sum_{j=i+1}^{n} u_{i,j} \sigma_{j,i}}
    \right)
  }

**Off-Diagonals**

.. math::

  \color{blue}{\sigma_{12}} &=
    (\color{magenta}{-u_{12} \sigma_{22}}
    \color{purple}{-u_{13} \sigma_{32} - u_{14} \sigma_{42}})
    / \color{red}{u_{11}} \\
  \color{blue}{\sigma_{13}} &=
    (\color{magenta}{-u_{12} \sigma_{23}}
    \color{purple}{-u_{13} \sigma_{33} - u_{14} \sigma_{43}})
    / \color{red}{u_{11}} \\
  \color{blue}{\sigma_{14}} &=
    (\color{magenta}{-u_{12} \sigma_{24}}
    \color{purple}{-u_{13} \sigma_{34} - u_{14} \sigma_{44}})
    / \color{red}{u_{11}} \\ \\
  \color{blue}{\sigma_{23}} &=
    (\color{magenta}{-u_{23} \sigma_{33}}
    \color{purple}{-u_{24} \sigma_{43}})
    / \color{red}{u_{22}} \\
  \color{blue}{\sigma_{24}} &=
    (\color{magenta}{-u_{23} \sigma_{34}}
    \color{purple}{-u_{24} \sigma_{44}})
    / \color{red}{u_{22}}  \\ \\
  \color{blue}{\sigma_{34}} &=
    (\color{magenta}{-u_{34} \sigma_{44}})
    / \color{red}{u_{33}}

.. math::

  \boxed{
    \color{blue}{\sigma_{il}} =
    \color{red}{\dfrac{1}{u_{ii}}}
    \left(
      \color{magenta}{-\sum_{j=i+1}^{l} u_{i,j} \sigma_{j,l}}
      \color{purple}{-\sum_{j=l+1}^{n} u_{i,j} \sigma_{j,l}}
    \right)
  }

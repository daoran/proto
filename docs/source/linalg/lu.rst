LU Decomposition
================

Lower–Upper (LU) decomposition or factorization factors a matrix as the
product of a lower triangular matrix and an upper triangular matrix. The
product sometimes includes a permutation matrix as well. LU decomposition can
be viewed as the matrix form of Gaussian elimination. Computers usually solve
square systems of linear equations using LU decomposition, and it is also a key
step when inverting a matrix or computing the determinant of a matrix.

Let :math:`\Mat{A}` be a square matrix. An LU factorization refers to the
factorization of :math:`\Mat{A}`, with proper row and/or column orderings or
permutations, into two factors a lower triangular matrix :math:`\Mat{L}` and an
upper triangular matrix :math:`\Mat{U}`:

.. math::
  \Mat{A} = \Mat{L} \Mat{U}

In the lower triangular matrix all elements above the diagonal are zero, in the
upper triangular matrix, all the elements below the diagonal are zero. For
example, for a :math:`3 \times 3` matrix :math:`\Mat{A}`, its :math:`\Mat{LU}`
decomposition looks like this:

.. math::

  \begin{bmatrix}
    a_{11} & a_{12} & a_{13} \\
    a_{21} & a_{22} & a_{23} \\
    a_{31} & a_{32} & a_{33}
  \end{bmatrix}
  =
  \begin{bmatrix}
    l_{11} & 0 & 0 \\
    l_{21} & l_{22} & 0 \\
    l_{31} & l_{32} & l_{33}
  \end{bmatrix}
  \begin{bmatrix}
    u_{11} & u_{12} & u_{13} \\
    0 & u_{22} & u_{23} \\
    0 & 0 & u_{33}
  \end{bmatrix}

Without a proper ordering or permutations in the matrix, the factorization may
fail to materialize. For example, it is easy to verify (by expanding the matrix
multiplication) that :math:`a_{11} = l_{11} u_{11}`. If :math:`a_{11} = 0`,
then at least one of :math:`l_{11}` and :math:`u_{11}` has to be zero, which
implies that either :math:`\Mat{L}` or :math:`\Mat{U}` is singular. This is
impossible if :math:`\Mat{A}` is non-singular (invertible). This is a
procedural problem. It can be removed by simply reordering the rows of A so
that the first element of the permuted matrix is non-zero. The same problem in
subsequent factorization steps can be removed the same way; see the basic
procedure below.

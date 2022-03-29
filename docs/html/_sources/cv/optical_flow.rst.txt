Optical Flow
============

Optical flow estimates the velocity of each image feature in successive
images of a scene. It makes the following explicit assumptions:

* Pixel intensity does not change between consecutive frames
* Displacement of features is small
* Features are within the same local neighbourhood

Let us consider a pixel, :math:`p`, in the first frame which has an intensity,
:math:`I(x, y, t)`, where it is a function of the pixel location, :math:`x` and
:math:`y`, and time, :math:`t`. If we apply the aforementioned assumptions, we
can say that the intensity of said pixel in the first frame to the second does
not change. Additionally, if there was a small displacement, :math:`dx` and
:math:`dy`, and small time difference, :math:`dt`, between images this can be
written in mathematical form as,

.. math::
  :label: brightness_constancy

  I(x, y, t) = I(x + dx, y + dy, t + dt).


This is known as the brightness constancy equation. To obtain the image
gradient and velocity of the pixel, we can use Taylor series approximation of
right-hand side of :eq:`brightness_constancy` to get,

.. math::

  \begin{align}
    &I(x + dx, y + dy, t + dt) \\ 
    &= I(x, y, t)
        + \dfrac{\partial{I}}{\partial{x}} dx
        + \dfrac{\partial{I}}{\partial{y}} dy
        + \dfrac{\partial{I}}{\partial{t}} dt
        + \dots
  \end{align}

removing common terms and dividing by :math:`dt` we get,

.. math::
  I_{x} v_{x} + I_{y} v_y + I_{t} = 0

or,

.. math::
  I_{x} v_{x} + I_{y} v_y = -I_{t}

where:

.. math::

  I_{x} = \dfrac{\partial I}{\partial x}
  ; \quad
  I_{y} = \dfrac{\partial I}{\partial y} \\
  v_{x} = \dfrac{dx}{dt}
  ; \quad
  v_y = \dfrac{dy}{dt}.


The image gradients along the x and y directions are :math:`I_{x}` and
:math:`I_{y}`, where :math:`I_{t}` is the image gradient along time, finally,
:math:`v_{x}` and :math:`v_{y}` are the pixel velocity in :math:`x` and
:math:`y` directions, which is unknown. The problem with with the above is that
it provides a single constraint with two degrees of freedom, and as such
requires at least one additional constraint to identify a solution.

The Lucas-Kanade method solves the aperture problem by introducing additional
conditions. This method assumes all pixels within a window centered around a
pixel :math:`p` will have similar motion, and that the window size is
configurable.  For example, a window size of :math:`3 \times 3` around the
pixel :math:`p`, the :math:`9` points within the window should have a similar
motion. Using the intensity inside the window must therefore satisfy,

.. math::

  \begin{align}
    I_{x}(p_1) v_{x}(p_1) &+ I_{y}(p_1) v_y = -I_{t}(p_1) \\
    I_{x}(p_1) v_{x}(p_2) &+ I_{y}(p_2) v_y = -I_{t}(p_2) \\
    & \enspace \vdots \\
    I_{x}(p_1) v_{x}(p_n) &+ I_{y}(p_n) v_y = -I_{t}(p_n)
  \end{align}

where :math:`p_{1}, p_{2} ,\dots , p_{n}` are the pixels in the window. This
can be re-written in matrix form :math:`\mathbf{A} \mathbf{x} = \mathbf{b}` as,

.. math::

  \begin{align}
    \mathbf{A} = \begin{bmatrix}
        I_{x}(p_{1}) & I_{y}(p_{1}) \\
        I_{x}(p_{2}) & I_{y}(p_{2}) \\
        \vdots & \vdots \\
        I_{x}(p_{n}) & I_{y}(p_{n})
    \end{bmatrix}
    \quad
    \mathbf{x} = \begin{bmatrix}
      v_{x} \\ v_{y} \\
    \end{bmatrix}
    \quad
    \mathbf{b} = \begin{bmatrix}
      -I_{t}(p_{1}) \\
      -I_{t}(p_{2}) \\
      \vdots \\
      -I_{t}(p_{n})
    \end{bmatrix}.
  \end{align}

The linear system of equations above is over-determined, therefore there is no
exact solution. To address this issue, a least squares method can be used to
approximate the solution by applying the ordinary least squares. For the system
:math:`\mathbf{A} \mathbf{x} = \mathbf{b}`, the least squares formula is
obtained by minimizing the following,

.. math::

  \begin{align}
    \underset{\mathbf{x}}{\text{argmin }}
      || \mathbf{A} \mathbf{x} - \mathbf{b} ||,
  \end{align}

the solution of which can be obtained by using *normal equations*,

.. math::

  \begin{align}
    \mathbf{A}^{T} \mathbf{A} \mathbf{x} &= \mathbf{A}^{T} \mathbf{b} \\
    \mathbf{x} &= (\mathbf{A}^{T} \mathbf{A})^{-1} \mathbf{A}^{T} \mathbf{b}
  \end{align}

where

.. math::

  \begin{bmatrix}
  v_{x} \\ v_{y}
  \end{bmatrix}
  =
  \begin{bmatrix}
    \sum_{i}{I_{x}(p_{i})}^2 & \sum_{i}{I_{x}(p_{i}) I_{y}(p_{i}) } \\
    \sum_{i}{I_{x}(p_{i}) I_{y}(p_{i})} & \sum_{i}{I_{y}(p_{i})}^2
  \end{bmatrix}^{-1}
  \begin{bmatrix}
    - \sum_{i}{I_{x}(p_{i}) I_{t}(p_{i})} \\
    - \sum_{i}{I_{y}(p_{i}) I_{t}(p_{i})}
  \end{bmatrix}

which is finally used to obtain the optical flow of pixel :math:`p`.

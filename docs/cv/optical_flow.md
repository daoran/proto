# Optical Flow

Optical flow estimates the velocity of each image feature in successive images
of a scene. It makes the following explicit assumptions:

- Pixel intensity does not change between consecutive frames}
- Displacement of features is small}
- Features are within the same local neighbourhood}

Let us consider a pixel, $p$, in the first frame which has an intensity, $I(x,
y, t)$, where it is a function of the pixel location, $x$ and $y$, and time,
$t$. If we apply the aforementioned assumptions, we can say that the intensity
of said pixel in the first frame to the second does not change. Additionally,
if there was a small displacement, $dx$ and $dy$, and small time difference,
$dt$, between images this can be written in mathematical form as,
\begin{equation}
  \label{eq:brightness_constancy}
  I(x, y, t) = I(x + dx, y + dy, t + dt).
\end{equation}
This is known as the brightness constancy equation. To obtain the image
gradient and velocity of the pixel, we can use Taylor series approximation of
right-hand side of \eqref{eq:brightness_constancy} to get,
\begin{equation}
  I(x + dx, y + dy, t + dt) = I(x, y, t)
    + \dfrac{\partial{I}}{\partial{x}} dx
    + \dfrac{\partial{I}}{\partial{y}} dy
    + \dfrac{\partial{I}}{\partial{t}} dt
    + \text{H.O.T},
\end{equation}
removing common terms and dividing by $dt$ we get,
\begin{equation}
  \label{eq:optical_flow}
  I_{x} v_{x} + I_{y} v_y + I_{t} = 0
\end{equation}
or,
\begin{equation}
  \label{eq:optical_flow_2}
  I_{x} v_{x} + I_{y} v_y = -I_{t}
\end{equation}
where:
\begin{align}
  I_{x} = \dfrac{\partial I}{\partial x}
  ; \quad
  I_{y} = \dfrac{\partial I}{\partial y} \nonumber \\
  v_{x} = \dfrac{dx}{dt}
  ; \quad
  v_y = \dfrac{dy}{dt}. \nonumber
\end{align}
The image gradients along the x and y directions are $I_{x}$ and $I_{y}$, where
$I_{t}$ is the image gradient along time, finally, $v_{x}$ and $v_{y}$ are the
pixel velocity in $x$ and $y$ directions, which is unknown. The problem with
\eqref{eq:optical_flow_2} is that it provides a single constraint with two
degrees of freedom, and as such requires at least one additional constraint to
identify a solution.

The Lucas-Kanade method solves the aperture problem by introducing additional
conditions. This method assumes all pixels within a window centered around a
pixel $p$ will have similar motion, and that the window size is configurable.
For example, a window size of $3 \times 3$ around the pixel $p$, the $9$ points
within the window should have a similar motion. Using
\eqref{eq:optical_flow_2}, the intensity inside the window must therefore
satisfy,
\begin{align}
  I_{x}(p_1) v_{x}(p_1) &+ I_{y}(p_1) v_y = -I_{t}(p_1) \nonumber \\
  I_{x}(p_1) v_{x}(p_2) &+ I_{y}(p_2) v_y = -I_{t}(p_2) \nonumber \\
  & \enspace \vdots \nonumber \\
  I_{x}(p_1) v_{x}(p_n) &+ I_{y}(p_n) v_y = -I_{t}(p_n) \nonumber
\end{align}
where $p_{1}, p_{2} ,\dots , p_{n}$ are the pixels in the window. This can be
re-written in matrix form $\mathbf{A} \mathbf{x} = \mathbf{b}$ as,
\begin{equation}
  \label{eq:lucas_kanade_1}
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
\end{equation}
The linear system of equations of \eqref{eq:lucas_kanade_1} is
over-determined, therefore there is no exact solution. To address this issue, a
least squares method can be used to approximate the solution by applying the
ordinary least squares. For the system $\mathbf{A} \mathbf{x} = \mathbf{b}$,
the least squares formula is obtained by minimizing the following,
\begin{equation}
  \underset{\mathbf{x}}{\text{argmin }} || \mathbf{A} \mathbf{x} - \mathbf{b} ||,
\end{equation}
the solution of which can be obtained by using *normal equations*,
\begin{align}
  \label{eq:normal_equations_1}
  \mathbf{A}^{T} \mathbf{A} \mathbf{x} &= \mathbf{A}^{T} \mathbf{b} \\
  \label{eq:normal_equations_2}
  \mathbf{x} &= (\mathbf{A}^{T} \mathbf{A})^{-1} \mathbf{A}^{T} \mathbf{b}.
\end{align}
Rewriting \eqref{eq:lucas_kanade_1} in the form of \eqref{eq:normal_equations_2} we get,
\begin{equation}
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
\end{equation}
which is finally used to obtain the optical flow of pixel $p$.

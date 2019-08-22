# Computer Vision

### Point on Line

\begin{equation}
  \Transpose{\Vec{x}} \Vec{l} = 0
\end{equation}


### Intersection of Lines

\begin{equation}
  \Vec{l} (\Vec{l} \times \Vec{l}')
  \Vec{l}' (\Vec{l} \times \Vec{l}')
  = 0 \\
  \Transpose{\Vec{l}} \Vec{x}
  = \Transpose{\Vec{l}}' \Vec{x}
  = 0
\end{equation}

\begin{equation}
  \Vec{x} = \Vec{l} \times \Vec{l}'
\end{equation}


### Plane

- A plane can be defined by the join between three points, or the join
  between a line and a point in general
- Two planes intersecting a unique line
- Three planes intersecting a unique point


### Three Points Define a Plane

Suppose you have three points $\Vec{X}_{1}$, $\Vec{X}_{2}$, $\Vec{X}_{3}$, and
are incident with a plane, $\boldsymbol{\pi}$ then each point satisfies

\begin{equation}
  \Transpose{\boldsymbol{\pi}} \Vec{X}_{i} = 0.
\end{equation}

By stacking each point as a matrix
\begin{align}
  \begin{bmatrix}
    \Transpose{\Vec{X}_{1}} \\
    \Transpose{\Vec{X}_{2}} \\
    \Transpose{\Vec{X}_{3}}
  \end{bmatrix} \boldsymbol{\pi} = 0
\end{align}
Since three points in general rare linearly independent, it follows that the
$3 \times 4$ matrix compsed of the points $\Vec{X}_{i}$ as rows has rank 3.

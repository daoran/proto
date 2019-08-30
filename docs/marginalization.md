# Marginalization

Let $\Mat{M}$ be a matrix that consists of block matrices $\Mat{A}$, $\Mat{B}$,
$\Mat{C}$, $\Mat{D}$,
\begin{equation}
	\Mat{M} =
	\begin{bmatrix}
		\Mat{A} & \Mat{B} \\
		\Mat{C} & \Mat{D}
	\end{bmatrix}
\end{equation}
if $\Mat{A}$ is invertible, the Schur complement of the block $\Mat{A}$ of the
matrix $\Mat{B}$ is the defined by
\begin{equation}
	\Mat{M}/\Mat{A} = \Mat{D} - \Mat{C} \Mat{A}^{-1} \Mat{B}.
\end{equation}
\begin{equation}
	\Mat{M}/\Mat{D} = \Mat{A} - \Mat{B} \Mat{D}^{-1} \Mat{C}.
\end{equation}

---

\begin{equation}
  \Mat{H} \delta\Vec{x} = \Vec{b}
\end{equation}

\begin{equation}
  \begin{bmatrix}
    \Mat{H}_{\mu\mu} & \Mat{H}_{\mu\lambda_{1}} \\
    \Mat{H}_{\lambda_{1}\mu} & \Mat{H}_{\lambda_{1}\lambda_{1}}
  \end{bmatrix}
  \begin{bmatrix}
    \delta\Vec{x}_{\mu} \\
    \delta\Vec{x}_{\lambda}
  \end{bmatrix}
  =
  \begin{bmatrix}
    \Vec{b}_{\mu} \\
    \Vec{b}_{\lambda}
  \end{bmatrix}
\end{equation}

\begin{equation}
  \Mat{H}^{*}_{\lambda_{1}\lambda_{1}}
  =
  \Mat{H}_{\lambda_{1}\lambda_{1}}
  - \Mat{H}_{\lambda_{1}\mu}
  \Mat{H}_{\mu\mu}^{-1}
  \Mat{H}_{\mu\lambda_{1}}
\end{equation}

\begin{equation}
  \Vec{b}^{*}_{\lambda_{1}}
  =
  \Vec{b}_{\lambda_{1}}
  - \Mat{H}_{\lambda_{1}\mu}
  \Mat{H}_{\mu\mu}^{-1}
  \Vec{b}_{\mu}
\end{equation}

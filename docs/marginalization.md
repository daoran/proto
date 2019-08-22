Marginalization
===============

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

# Marginalization

## Shur's Complement

Let $\Mat{M}$ be a matrix that consists of block matrices $\Mat{A}$, $\Mat{B}$,
$\Mat{C}$, $\Mat{D}$,
\begin{equation}
	\Mat{M} =
	\begin{bmatrix}
		\Mat{A} & \Mat{B} \\
		\Mat{C} & \Mat{D}
	\end{bmatrix}
\end{equation}
if $\Mat{A}$ is invertible, the Schur's complement of the block $\Mat{A}$ of the
matrix $\Mat{B}$ is the defined by
\begin{equation}
	\Mat{M}/\Mat{A} = \Mat{D} - \Mat{C} \Mat{A}^{-1} \Mat{B}.
\end{equation}
\begin{equation}
	\Mat{M}/\Mat{D} = \Mat{A} - \Mat{B} \Mat{D}^{-1} \Mat{C}.
\end{equation}

---

## Using Shur's Complement for marginalization

In a Gauss-Newton system,
\begin{equation}
  \Mat{H} \delta\state = \Vec{b} ,
\end{equation}
it so happens that Schur's complement can be used to both invert and
marginalize out the old states. First let $\state_\mu$ be the states to be
marginalized out, $\state_{\lambda}$ be the set of states related to those by
error terms, and $\state_{\rho}$ be the set of remaining states. Partitioning
the Hessian, error state and R.H.S of the Gauss-Newton system gives:
\begin{equation}
  \begin{bmatrix}
    \Mat{H}_{\mu\mu} & \Mat{H}_{\mu\lambda_{1}} \\
    \Mat{H}_{\lambda_{1}\mu} & \Mat{H}_{\lambda_{1}\lambda_{1}}
  \end{bmatrix}
  \begin{bmatrix}
    \delta\state_{\mu} \\
    \delta\state_{\lambda}
  \end{bmatrix}
  =
  \begin{bmatrix}
    \Vec{b}_{\mu} \\
    \Vec{b}_{\lambda}
  \end{bmatrix}
\end{equation}
and applying the Shur complement operation yields:
\begin{align}
  \Mat{H}^{\ast}_{\lambda_{1}\lambda_{1}}
  &=
  \Mat{H}_{\lambda_{1}\lambda_{1}} -
  \Mat{H}_{\lambda_{1}\mu}
  \Mat{H}_{\mu\mu}^{-1}
  \Mat{H}_{\mu\lambda_{1}}
	\\
	\Vec{b}^{\ast}_{\lambda_{1}}
  &=
  \Vec{b}_{\lambda_{1}} -
  \Mat{H}_{\lambda_{1}\mu}
  \Mat{H}_{\mu\mu}^{-1}
  \Vec{b}_{\mu}
\end{align}
where $\Vec{b}^{\ast}_{\lambda_{1}}$ and
$\Mat{H}^{\ast}_{\lambda_{1}\lambda_{1}}$ are non-linear functions of
$\state_\lambda$ and $\state_\mu$.

The finite deviation $\Delta{\chi}= \Phi^{-1}(\log(\bar{\state} \boxplus
\state_{0}^{-1}))$ represents state updates that occur after marginalization,
where $\bar{\state}$ is our current estimate for $\state$. Introducing and
approximating the R.H.S of the Gauss-Newton equation with $\Delta{\chi}$ and
the first order Taylor series results in,
\begin{equation}
	\label{eq:gn_rhs_v2}
	\Vec{b} + \dfrac{\delta{b}}{\delta{\Delta{\chi}}} \bigg\rvert_{\state_{0}}
    = \Vec{b} - \Mat{H} \Delta{\chi}.
\end{equation}
Partioning \eqref{eq:gn_rhs_v2} into $\mu$ and $\lambda$,
\begin{equation}
	\label{eq:gn_rhs_v2_partitioned}
	\begin{bmatrix}
		\Vec{b}_{\mu} \\ \Vec{b}_{\lambda_{1}}
	\end{bmatrix}
		=
		\begin{bmatrix}
			\Vec{b}_{\mu, 0} \\ \Vec{b}_{\lambda_{1}, 0}
		\end{bmatrix}
		-
		\begin{bmatrix}
			\Mat{H}_{\mu\mu} & \Mat{H}_{\mu\lambda_{1}} \\
			\Mat{H}_{\lambda_{1}\mu} & \Mat{H}_{\lambda_{1}\lambda_{1}}
		\end{bmatrix}
		\begin{bmatrix}
			\Delta{\chi}_{\mu} \\
			\Delta{\chi}_{\lambda_{1}}
		\end{bmatrix}.
\end{equation}
Substituting in \eqref{eq:gn_rhs_v2_partitioned} to the R.H.S of the
Gauss-Newton system, $\Mat{H} \delta{\state} = \Vec{b}$, results in,
\begin{equation}
	\Vec{b}^{\ast}_{\lambda_{1}} =
		\underbrace{
			\Vec{b}_{\lambda_{1}, 0} -
			\Mat{H}_{\lambda_{1}\mu}
			\Mat{H}_{\mu\mu}^{-1}
			\Vec{b}_{\mu, 0}
		}_{\Vec{b}^{\ast}_{\lambda_{1}, 0}}
		-
		\Mat{H}^{\ast}_{\lambda_{1}\lambda_{1}}
		\Delta{\chi}_{\lambda_{1}}.
\end{equation}

---

Let us consider the following scenario. A state vector, $\state$, during the
time interval $[0, k]$ will contain $m$ old states to be marginalized out and
$r$ remain states which we wish to keep. i.e. $\state =
[\state_{m}^{\transpose} \quad \state_{r}^{\transpose}]^{\transpose}$. Then the
cost function, $c(\cdot)$, can be written as a function of $\state$ at time $k$
as,
\begin{align}
	\begin{split}
		c(\state_{k}) &= c(\state_{m}, \state_{r}) \\
									&= c(\state_{m}) + c(\state_{r}).
	\end{split}
	\label{eq:ba_cost_fn}
\end{align}
The intuition behind \eqref{eq:ba_cost_fn} is since the state at time $k$ can
be partitioned into $m$ and $r$, the cost can also be decomposed. Utilizing
this property, the multi-variate optimization can also be decomposed as
follows,
\begin{align}
	\min_{\state_{m}, \state_{r}} c(\state_{m}, \state_{r})
		&= \min_{\state_{r}} (\min_{\state_{m}} c(\state_{m}, \state_{r})) \\
		&= \min_{\state_{r}} (c(\state_{r}) + \min_{\state_{m}} c(\state_{m})) .
		\label{eq:ba_cost_decomposed}
\end{align}
Equation \eqref{eq:ba_cost_decomposed} shows the minimization problem can be
solved by first optimizing for the states $\state_{m}$, and then forming a
prior towards the problem of solving for $\state_{r}$. The reformulation of the
minimization problem entails no approximation.

The Gauss Newton system for solving \eqref{eq:ba_cost_decomposed} is,
\begin{equation}
	\Mat{H} \Delta{\state} = \Vec{b}.
\end{equation}
Partitioning the $\Mat{H}$ matrix into $\mu$ for states to be marginalized out,
and $\lambda$ for states to remain,
\begin{equation}
  \begin{bmatrix}
    \Mat{H}_{\mu\mu} & \Mat{H}_{\mu\lambda_{1}} \\
    \Mat{H}_{\lambda_{1}\mu} & \Mat{H}_{\lambda_{1}\lambda_{1}}
  \end{bmatrix}
  \begin{bmatrix}
    \delta\state_{\mu} \\
    \delta\state_{\lambda}
  \end{bmatrix}
  =
  \begin{bmatrix}
    \Vec{b}_{\mu} \\
    \Vec{b}_{\lambda}
  \end{bmatrix}
\end{equation}
Solving for $\Delta{\state}$ via the Schur's Complement operation,
\begin{align}
  \Mat{H}^{\ast}_{\lambda_{1}\lambda_{1}}
  &=
  \Mat{H}_{\lambda_{1}\lambda_{1}} -
  \Mat{H}_{\lambda_{1}\mu}
  \Mat{H}_{\mu\mu}^{-1}
  \Mat{H}_{\mu\lambda_{1}}
	\\
	\Vec{b}^{\ast}_{\lambda_{1}}
  &=
  \Vec{b}_{\lambda_{1}} -
  \Mat{H}_{\lambda_{1}\mu}
  \Mat{H}_{\mu\mu}^{-1}
  \Vec{b}_{\mu}
\end{align}
where $\Vec{b}^{\ast}_{\lambda_{1}}$ and
$\Mat{H}^{\ast}_{\lambda_{1}\lambda_{1}}$ are non-linear functions of
$\state_\lambda$ and $\state_\mu$.




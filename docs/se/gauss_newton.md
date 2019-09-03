# Gauss Newton

\begin{align}
  \min_{\Vec{x}} J(\Vec{x})
		&=
			\dfrac{1}{2}
			\sum_{i}
		  \Vec{e}_{i}^{\transpose} \Mat{W} \Vec{e}_{i} \\
		&=
			\dfrac{1}{2} \enspace
			\Vec{e}_{i}^{\transpose}(\Vec{x})
			\Mat{W}
			\Vec{e}_{i}(\Vec{x})
\end{align}

Since the error function, $\Vec{e}(\Vec{x})$, is non-linear, it is
approximated with the first-order Taylor series,
\begin{equation}
	\Vec{e}(\Vec{x})
		\approx
			\Vec{e}(\bar{\Vec{x}}) +
			\Mat{E}(\bar{\Vec{x}}) \Delta\Vec{x}
\end{equation}
where $\Mat{E}(\bar{\Vec{x}}) = \dfrac{\delta\Vec{e}(\Vec{x})}{\delta\Vec{x}}
\bigg\rvert_{\Vec{x}_{k}}$ and $\Delta{\Vec{x}} = \Vec{x} - \bar{\Vec{x}}$.

\begin{equation}
	\dfrac{\delta{J}}{\delta{\Vec{x}}} =
		\dfrac{\delta{J}}{\delta{\Vec{e}}}
		\dfrac{\delta{\Vec{e}}}{\delta{\Vec{x}}}
\end{equation}

\begin{align}
	\dfrac{\delta{J}}{\delta{\Vec{e}}} &=
		\dfrac{1}{2} \Vec{e}^{\transpose}(\Vec{x}) \Mat{W} \Vec{e}(\Vec{x}) =
		\Vec{e}^{\transpose}(\Vec{x}) \Mat{W} \\
	%
	\dfrac{\delta{\Vec{e}}}{\delta{\Vec{x}}} &=
		\Vec{e}(\bar{\Vec{x}}) +
		\Mat{E}(\bar{\Vec{x}}) \Delta\Vec{x} =
		\Mat{E}(\bar{\Vec{x}})
\end{align}

\begin{align}
	\dfrac{\delta{J}}{\delta{\Vec{x}}}
		&=
			(\Vec{e}^{\transpose}(\Vec{x}) \Mat{W}) (\Mat{E}(\bar{\Vec{x}})) \\
		% Line 2
		&=
			(
				\Vec{e}(\bar{\Vec{x}}) + \Mat{E}(\bar{\Vec{x}}) \Delta\Vec{x}
			)^{\transpose} \Mat{W}
			\Mat{E}(\bar{\Vec{x}}) \\
		% Line 3
		&=
			\Vec{e}^{\transpose}(\bar{\Vec{x}}) \Mat{W} \Mat{E}(\bar{\Vec{x}})
			+ \Delta\Vec{x}^{\transpose}
				\Mat{E}(\bar{\Vec{x}})^{\transpose} \Mat{W} \Mat{E}(\bar{\Vec{x}})
			= 0 \\
\end{align}

\begin{align}
		% Line 4
		\Delta\Vec{x}^{\transpose}
			\Mat{E}(\bar{\Vec{x}})^{\transpose} \Mat{W} \Mat{E}(\bar{\Vec{x}})
		&=
			- \Vec{e}^{\transpose}(\bar{\Vec{x}}) \Mat{W} \Mat{E}(\bar{\Vec{x}}) \\
		% Line 5
		\underbrace{
			\Mat{E}(\bar{\Vec{x}})^{\transpose} \Mat{W} \Mat{E}(\bar{\Vec{x}})
		}_{\Mat{H}}
			\Delta\Vec{x}
		&=
		\underbrace{
			- \Mat{E}(\bar{\Vec{x}})^{\transpose} \Mat{W} \Vec{e}(\bar{\Vec{x}})
		}_{\Vec{b}}
\end{align}

Solve the normal equations $\Mat{H}\Delta\Vec{x} = \Vec{b}$ for $\Delta\Vec{x}$
using the Cholesky or QR-decompositon. Once $\Delta\Vec{x}$ is found the best
estimate $\bar{\Vec{x}}$ can be updated via,
\begin{equation}
	\bar{\Vec{x}}_{k + 1} = \bar{\Vec{x}}_{k} + \Delta\Vec{x} .
\end{equation}

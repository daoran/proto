Outerloop Linear MPC for MAV
============================

Position tracking is achieved by means of a cascaded connection of a Model
Predictive Controller (MPC) for the MAV position and a PID controller for its
attitude. This approach is motivated by the fact the majority of commercially
available flight controllers come with a pre-implemented attitude controller
which requires little or even no tuning, enabling easy adaptation to a wide
range of platforms.



Linear Model
------------

The linear model of the MAV in navigation frame :math:`\frame_{N}` (or body frame)
is,

.. math::

    \dVel_{N} = \begin{bmatrix}
        g \dot{\theta} - c_{x} \dot{x} \\
        -g \dot{\phi} - c_{y} \dot{y} \\
        T - c_{z} \dot{z}
    \end{bmatrix}

The closed loop attitude dynamics is modelled as a first order system. Namely
the roll and pitch are:

.. math::

  \begin{align}
    \ddot{\theta} &=
      -b_{\ddot{\theta}\theta} \theta
      -b_{\ddot{\theta}\dot{\theta}} \dot{\theta}
      +b_{\theta^{r}} \theta^{r} \\
    \ddot{\phi} &=
      -b_{\ddot{\phi}\phi} \phi
      -b_{\ddot{\phi}\dot{\phi}} \dot{\phi}
      +b_{\phi^{r}} \phi^{r}
  \end{align}

where :math:`b_{(\cdot)}` are constants of the first order system. The values
of these constants are obtained by performing system identification of the MAV.
Yaw is omitted above because the reference yaw command :math:`\psi^{r}` will be
passed directly to the inner-loop attitude control, therefore it is not
considered here in the outer-loop linear MPC controller.

The state vector :math:`\state` is,

.. math::

  \state = \begin{bmatrix}
    x \enspace \dot{x} \enspace \theta \enspace \dot{\theta}
    \enspace \enspace
    y \enspace \dot{y} \enspace \phi \enspace \dot{\phi} \enspace
    \enspace \enspace
    z \enspace \dot{z}
  \end{bmatrix} \in \real^{10}

The input vector :math:`\u` to the linear model contains reference roll :math:`\theta^{r}`,
pitch :math:`\phi^{r}` and thrust `T^{r}`, or written as

.. math::

  \u = \begin{bmatrix}
    \theta^{r}
    \enspace \phi^{r}
    \enspace T^{r}
  \end{bmatrix}

Time-invariant state space representation:

.. math::

  \begin{align}
    \dot{\state} &=
        \underbrace{
            \begin{bmatrix}
                \Mat{A}_{\text{LON}} & \Vec{0} & \Vec{0} \\ 
                \Vec{0} & \Mat{A}_{\text{LAT}} & \Vec{0} \\ 
                \Vec{0} & \Vec{0} & \Mat{A}_{\text{ALT}}
            \end{bmatrix}
        }_{\Mat{A}}
        \Vec{x}
        +
        \underbrace{
            \begin{bmatrix}
                \Mat{B}_{\text{LON}} & \Vec{0} & \Vec{0} \\ 
                \Vec{0} & \Mat{B}_{\text{LAT}} & \Vec{0} \\ 
                \Vec{0} & \Vec{0} & \Mat{B}_{\text{ALT}}
            \end{bmatrix}
        }_{\Mat{B}} \Vec{u} \\
    \Vec{y} &= \Mat{C} \Vec{x}
  \end{align}

where

.. math::

  \begin{align}
    \Mat{A}_{\text{LON}} &= \begin{bmatrix}
        0 & 1 & 0 \\ 
        0 & -c_{x} & g \\ 
        0 & 0 & -b_{\theta}
    \end{bmatrix}
    ,& 
    &\quad
    &\Mat{B}_{\text{LON}} &= \begin{bmatrix}
        0 \\ 
        0 \\
        b_{\theta^{r}}
    \end{bmatrix} \\
    \Mat{A}_{\text{LAT}} &= \begin{bmatrix}
        0 & 1 & 0 \\ 
        0 & -c_{x} & -g \\ 
        0 & 0 & -b_{\phi}
    \end{bmatrix}
    ,& 
    &\quad
    &\Mat{B}_{\text{LAT}} &= \begin{bmatrix}
        0 \\ 
        0 \\
        b_{\phi^{r}}
    \end{bmatrix} \\
    \Mat{A}_{\text{ALT}} &= \begin{bmatrix}
        0 & 1 \\
        0 & -c_{z}
    \end{bmatrix}
    ,& 
    &\quad
    &\Mat{B}_{\text{ALT}} &= \begin{bmatrix}
        0 \\ 
        1
    \end{bmatrix}
  \end{align}

.. math::
    % C Matrix
    \Mat{C} = \Mat{I}_{8 \times 8} .


Since the controller is implemented in discrete time, the equations are
discretized using zero order hold for the input `u`. The discrete equivalent of
the matrices :math:`\Mat{A}`, :math:`\Mat{B}` and :math:`\Mat{C}` can be obtained by.

.. math::

  \begin{align}
    \Mat{A}_{d} &= e^{\Mat{A} dt} \\
    \Mat{B}_{d} &= \left( \int_{0}^{dt} e^{\Mat{A}\tau} d\tau \right) \Mat{B} \\
    \Mat{C}_{d} &= \Mat{C}
  \end{align}

The discretization step `dt` coincides with the control update rate and was set
to 50ms.



MPC Formulated as a QP Problem
------------------------------

Our goal is to obtain an optimal input sequence :math:`\bar{\u}^{\ast} =
\u_{0}^{\ast} \dots \u_{N-1}^{\ast}` which is the solution of the following
optimization problem:

.. math::
  \begin{align}
  	& \bar{\u}^{\ast} = \Argmin{\u_{0} \dots \u_{N - 1}} J, \\
  	s.t. : \enspace
  			& \state_{k + 1} = \Mat{A}_{d} \state_{k} + \Mat{B}_{d} \u_{k}, \\
  			& \mathbf{y}_{k} = \Mat{C}_{d} \state_{k}, \\
  			& \mathbf{x}_{0} = \hat{\Mat{x}}_{0}, \\
  			& \hat{\u}_{\text{min}} \leq \u \leq \u_{\text{max}}
  \end{align}

.. math::

	J = \sum_{k = 0}^{N - 1}
  \left(
    \Norm{\Mat{Q}_{k + 1} (\Vec{y}_{k + 1} - \Vec{s}_{k + 1}^{y})}_{2}^{2}
    + \Norm{\Mat{R}_{k} (\Vec{u}_{k} - \Vec{s}_{k}^{u})}_{2}^{2}
  \right)


where:

* :math:`\state_{k + 1} \in \real^{n}`: system state at time :math:`k`
* :math:`\hat{\state}_{0} \in \real^{n}`: estimated state at time 0
* :math:`\Vec{y}_{k} \in \real^{p}`: system output at time :math:`k`
* :math:`\Vec{u}_{k}`: system input at time :math:`k`
* :math:`\Vec{s}_{k}^{y} \in \real^{p}`: reference output at time :math:`k`
  (given)
* :math:`\Vec{s}_{k}^{u} \in \real^{m}`: reference input at time :math:`k`
  (given)
* :math:`\Mat{R}_{k} \in \real^{m \times m}`: input gain matrix (tuning
  parameters)
* :math:`\Mat{Q}_{k} \in \real^{n \times n}`: output gain matrix (tuning
  parameters)

By concatenating the two squared 2-norms that appear in the cost function `J`,
we can rewrite it as:

.. math::

    J = \Norm{\begin{matrix}
        \Mat{Q}_{1} (\Vec{y}_{1} - \Vec{s}_{1}^{y}) \\
        \Mat{Q}_{2} (\Vec{y}_{2} - \Vec{s}_{2}^{y}) \\
        \vdots \\
        \Mat{Q}_{N} (\Vec{y}_{N} - \Vec{s}_{N}^{y}) \\
        \Mat{R}_{0} (\Vec{u}_{0} - \Vec{s}_{0}^{u}) \\
        \Mat{R}_{1} (\Vec{u}_{1} - \Vec{s}_{1}^{u}) \\
        \vdots \\
        \; \Mat{R}_{N-1} (\Vec{u}_{N-1} - \Vec{s}_{N-1}^{u}) \\
    \end{matrix}}_{2}^{2}

and stacking the :math:`\Mat{Q}` and :math:`\Mat{R}` as,

.. math::

    J = \Norm{
            \begin{matrix}
                \; \bar{\Mat{Q}}(\bar{\Vec{y}} - \bar{\Vec{s}}^{y}) \\
                \; \bar{\Mat{R}}(\bar{\Vec{u}} - \bar{\Vec{s}}^{u})
            \end{matrix}
        }_{2}^{2}.

The problem with the current formulation is the equality constraints
:math:`\Vec{x}_{k + 1}`, :math:`\Vec{y}_{k}` and :math:`\state_{0}` may not be valid in practice
due to imperfect model, and/or sensor measurement noise. If the equality
constraints are invalid the optimized solution will not be feasible. Instead,
the equality constraints can be eliminated by rewriting :math:`\bar{\Vec{y}}` to
depend only on the initial state :math:`\state_{0}` instead of :math:`\state_{k - 1}`. In
other words from this,

.. math::

  \begin{align}
    \state_{1} &= \Mat{A}_{d} \state_{0} + \Mat{B}_{d} \Vec{u}_{0} \\
    \state_{2} &= \Mat{A}_{d} \state_{1} + \Mat{B}_{d} \Vec{u}_{1} \\
    \state_{3} &= \Mat{A}_{d} \state_{2} + \Mat{B}_{d} \Vec{u}_{2} \\
    & \qquad \qquad \vdots \\
    \state_{N} &= \Mat{A}_{d} \state_{N-1} + \Mat{B}_{d} \Vec{u}_{N-1}
  \end{align}

to this,

.. math::

  \begin{align}
    \state_{1} &= \Mat{A}_{d} \state_{0} + \Mat{B}_{d} \Vec{u}_{0} \\
    \state_{2} &= \Mat{A}_{d}^{2} \state_{0}
    		+ \Mat{A}_{d} \Mat{B}_{d} \Vec{u}_{0}
    		+ \Mat{B}_{d} \Vec{u}_{1} \\
    \state_{3} &= \Mat{A}_{d}^{3} \state_{0}
    		+ \Mat{A}_{d}^{2} \Mat{B}_{d} \Vec{u}_{0}
    		+ \Mat{A}_{d} \Mat{B}_{d} \Vec{u}_{1}
    		+ \Mat{B}_{d} \Vec{u}_{2} \\
    & \qquad \qquad \qquad \vdots \\
    \state_{N} &= \Mat{A}_{d}^{N} \state_{0}
    		+ \Mat{A}_{d}^{N-1}\Mat{B}_{d} \Vec{u}_{0}
    		+ \dots
    		+ \Mat{B} \Vec{u}_{N-1}
    \bar{\state} = \mathbf{\Phi} \state_{0} + \mathbf{\Gamma} \bar{\Vec{u}}
  \end{align}

where

.. math::
    % xbar
    &\bar{\state} = \begin{bmatrix}
        \state_{1} \\ 
        \state_{2} \\ 
        \vdots \\
        \state_{N}
    \end{bmatrix},
    % Phi
    &\mathbf{\Phi} = \begin{bmatrix}
        \Mat{A}_{d} \\
        \Mat{A}_{d}^{2} \\
        \vdots \\
        \Mat{A}_{d}^{N}
    \end{bmatrix} \\
    % Gamma
    &\mathbf{\Gamma} = \begin{bmatrix}
        \Mat{B}_{d} & \mathbf{0} & \dots & \mathbf{0} \\
        \Mat{A}_{d} \Mat{B}_{d} & \Mat{B}_{d} & \dots & \mathbf{0} \\
        \vdots & \vdots & \ddots & \vdots \\
        \Mat{A}_{d}^{N-1} \Mat{B}_{d} & \Mat{A}_{d}^{N-2} \Mat{B}_{d} & \dots & \Mat{B}_{d} \\
    \end{bmatrix},
    % ubar
    &\bar{\Vec{u}} = \begin{bmatrix}
        \Vec{u}_{0} \\ 
        \Vec{u}_{1} \\ 
        \vdots \\
        \Vec{u}_{N-1}
    \end{bmatrix} .

Rewriting :math:`\bar{\Vec{y}}` with the above,

.. math::

    \bar{\Vec{y}} = \bar{\Mat{C}} \bar{\Vec{x}}
        = \bar{\Mat{C}} \mathbf{\Phi} \hat{\state}
        + \bar{\Mat{C}} \mathbf{\Gamma} \bar{\Vec{u}},

and substituting into the cost function `J`, collect the :math:`\bar{\Vec{u}}`
terms and rearrange so that it is in the form of :math:`\Mat{A}\Vec{\state} -
\Vec{b}`,

.. math::

  \begin{align}
    % Line 1
    J &= \Norm{\begin{matrix}
        \bar{\Mat{Q}} (\bar{\Mat{C}} \mathbf{\Phi} \state_{0}
            + \mathbf{\Gamma} \bar{\Vec{u}}
            - \bar{\Vec{s}}^{y}) \\
        \bar{\Mat{R}} (\bar{\Vec{u}} - \bar{\Vec{s}}^{u})
    \end{matrix}}_{2}^{2} \\
    % Line 2
    &= \Norm{\begin{matrix}
        \bar{\Mat{Q}} \bar{\Mat{C}} \mathbf{\Phi} \state_{0}
            + \bar{\Mat{Q}} \mathbf{\Gamma} \bar{\Vec{u}}
            - \bar{\Mat{Q}} \bar{\Vec{s}}^{y} \\
        \bar{\Mat{R}} \bar{\Vec{u}} - \bar{\Mat{R}} \bar{\Vec{s}}^{u}
    \end{matrix}}_{2}^{2} \\
    % Line 3
    &= \Norm{
        \underbrace{
            \left(\begin{matrix}
                \bar{\Mat{Q}} \bar{\Mat{C}} \mathbf{\Gamma} \\
                \bar{\Mat{R}}
            \end{matrix}\right) \bar{\Vec{u}}
        -
            \left(\begin{matrix}
                \bar{\Mat{Q}} \bar{\Vec{s}}^{y}
                  + \bar{\Mat{Q}} \bar{\Mat{C}}
                    \mathbf{\Phi} \state_{0} \\
                \bar{\Mat{R}} \bar{\Vec{s}}^{u}
            \end{matrix}\right)
        }_{\Mat{A}\Vec{x} - \Vec{b}}
    }_{2}^{2}
  \end{align}

then expanding the equation out and ignoring the constant term (i.e.
:math:`\Vec{b}^{\transpose}\Vec{b}`) gives,

.. math::

  \begin{align}
    J =
    \underbrace{
        \bar{\Vec{u}}^{\transpose}
        \left(\begin{matrix}
            \bar{\Mat{Q}} \bar{\Mat{C}} \mathbf{\Gamma} \\
            \bar{\Mat{R}}
        \end{matrix}\right)^{\transpose}
        \left(\begin{matrix}
            \bar{\Mat{Q}} \bar{\Mat{C}} \mathbf{\Gamma} \\
            \bar{\Mat{R}}
        \end{matrix}\right)
        \bar{\Vec{u}} \\
        - 2
        \left(\begin{matrix}
            \bar{\Mat{Q}} \bar{\Vec{s}}^{y}
            + \bar{\Mat{Q}} \bar{\Mat{C}} \mathbf{\Phi} \state_{0} \\
            \bar{\Mat{R}} \bar{\Vec{s}}^{u}
        \end{matrix}\right)^{\transpose}
        \left(\begin{matrix}
            \bar{\Mat{Q}} \bar{\Mat{C}} \mathbf{\Gamma} \\
            \bar{\Mat{R}}
        \end{matrix}\right)
        \bar{\Vec{u}}
    }_{
        \Vec{x}^{\transpose} \Mat{A}^{\transpose}\Mat{A}\Vec{x}
        - 2 \Vec{b}^{\transpose} \Mat{A} \Vec{x}
    }
  \end{align}

With the cost function in quadratic form, the optimization problem is now
transformed into the following equivalent QP with inequality constraints:

.. math::

  \begin{align}
	& \bar{\u}^{\ast} = \Argmin{\u_{0} \dots \u_{N - 1}}
			J , \\
	s.t. :
			& \hat{\u}_{\text{min}} \leq \u \leq \u_{\text{max}}
  \end{align}

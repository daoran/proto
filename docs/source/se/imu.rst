IMU Preintegration
==================

IMU Motion Model
----------------

The accelerometer and gyroscope measurements from the IMU are given by:

.. math::
   \accMeas &= \acc + \accBias + \rot_{SW} \gravity + \accNoise \\
   \gyrMeas &= \gyr + \gyrBias + \gyrNoise

Given two time instances :math:`k` and :math:`k+1`

.. math::
  \begin{align}
    % Position
    \pos_{{WS}_{k+1}} &=
      \pos_{{WS}_{k}}
      + \vel_{{WS}_{k+1}} \Delta{t_{k,k+1}}
      + \iint_{t \in [t_k, t_{k+1}]}
        \rot_{WS} (\accMeas - \accBias - \accNoise) - \gravity
        \enspace dt^{2} \\
    % Velocity
    \vel_{{WS}_{k+1}} &=
      \vel_{{WS}_{k}}
      + \int_{t \in [t_k, t_{k+1}]}
        \rot_{WS} (\accMeas - \accBias - \accNoise) - \gravity
        \enspace dt \\
    % Orientation
    \quat_{{WS}_{k+1}} &= \quat_{{WS}_{k}} \otimes
      \int_{t \in [t_k, t_{k+1}]}
        \dfrac{1}{2} \boldsymbol{\Omega}(\gyrMeas - \gyrBias - \gyrNoise)
        \enspace \quat_{S_{k}S_{t}} \enspace dt \\
  \end{align}


change reference frame from world frame to local sensor frame at time :math:`k`

.. math::
  \begin{align}
    % Position
    \rot_{{S_{k}W}} \pos_{{WS}_{k+1}} &=
      \rot_{{S_{k}W}} \left(
        \pos_{{WS}_{k}}
        + \vel_{{WS}_{k+1}} \Delta{t_{k,k+1}}
        - \dfrac{1}{2} \gravity \Delta{t^{2}_{k,k+1}}
      \right)
      + \boldsymbol{\alpha}_{k,k+1} \\
    % Velocity
    \rot_{{S_{k}W}} \vel_{{WS}_{k+1}} &=
      \rot_{{S_{k}W}} \left(
        \vel_{{WS}_{k}}
        - \dfrac{1}{2} \gravity \Delta{t_{k,k+1}}
      \right)
      + \boldsymbol{\beta}_{k,k+1} \\
    % Orientation
    \quat_{{S_{k}W}} \otimes \quat_{{WS}_{k+1}} &=
      \boldsymbol{\gamma}_{k,k+1} \\
  \end{align}

where,

.. math::
  \begin{align}
    % Position
    \boldsymbol{\alpha}_{k,k+1} &=
      \iint_{t \in [t_k, t_{k+1}]}
        \rot_{{S_{k}S_{t}}} (\accMeas - \accBias - \accNoise) - \gravity
        \enspace dt^{2} \\
    % Orientation
      \boldsymbol{\gamma}_{k,k+1} &= \int_{t \in [t_k, t_{k+1}]}
        \dfrac{1}{2} \boldsymbol{\Omega}(\gyrMeas - \gyrBias - \gyrNoise)
        \enspace \quat_{S_{k}S_{t}} \enspace dt \\
    % Velocity
    \boldsymbol{\beta}_{k,k+1} &=
      \int_{t \in [t_k, t_{k+1}]}
        \rot_{{S_{k}S_{t}}} (\accMeas - \accBias - \accNoise) - \gravity
        \enspace dt \\
  \end{align}

these are the terms that can be pre-integrated to reduce computational
complexity. For discrete time implementation, different numerical integration
methods such as Euler, midpoint and Ruge-Kutta integration can be used. Here,
Euler integration is chosen. At the start, :math:`\boldsymbol{\alpha}_{k,k}`,
:math:`\boldsymbol{\beta}_{k,k}` is :math:`\mathbf{0}` and
:math:`\boldsymbol{\gamma}_{k,k}` is identity quaternion.

.. math::
  \begin{align}
    % Position
    \hat{\boldsymbol{\alpha}}_{S_{k}S_{i + 1}} &=
      \hat{\boldsymbol{\alpha}}_{S_{k}S_{i}}
      + \hat{\boldsymbol{\beta}}_{S_{k}S_{i}} \delta t
      + \dfrac{1}{2}
        \rot(\hat{\boldsymbol{\gamma}}_{S_{k}S_{i}})
        (\accMeas - \accBias) \delta{t}^{2} \\
    % Velocity
    \hat{\boldsymbol{\beta}}_{S_{k}S_{i + 1}} &=
      \hat{\boldsymbol{\beta}}_{S_{k}S_{i}}
      + \rot(\hat{\boldsymbol{\gamma}}_{S_{k}S_{i}})
        (\accMeas - \accBias) \delta{t} \\
    % Orientation
      \hat{\boldsymbol{\gamma}}_{S_{k}S_{i+1}} &=
        \hat{\boldsymbol{\gamma}}_{S_{k}S_{i}}
        \otimes
        \begin{bmatrix}
          1 \\ 
          \frac{1}{2} (\gyrMeas - \gyrBias) \delta{t}
        \end{bmatrix} \\
  \end{align}

The linearized dynamics of the error state for :math:`\boldsymbol{\alpha}`,
:math:`\boldsymbol{\beta}`, :math:`\boldsymbol{\gamma}`:

.. math::
  \begin{align}
    % Position
    \dotdalpha &= \dbeta \\
    % Velocity
    \dotdbeta
      &=
      -\rot_{{S_{k}S_{t}}} \Skew{\accMeas - \accBias} \dtheta
      -\rot_{{S_{k}S_{t}}} \delta{\accBias}
      -\rot{{S_{k}S_{t}}} \accNoise \\
    % Orientation
    \dotdtheta
      &=
      -\Skew{\gyrMeas - \gyrBias} \dtheta
      - \delta{\gyrBias}
      - \gyrNoise \\
  \end{align}

.. math::
  \Vec{x} = \begin{bmatrix}
    \boldsymbol{\alpha} \\
    \boldsymbol{\beta} \\
    \boldsymbol{\gamma} \\
    \accBias \\
    \gyrBias
  \end{bmatrix}
  \enspace
  \delta{\Vec{x}} = \begin{bmatrix}
    \dalpha \\
    \dbeta \\
    \dgamma \\
    \delta{\accBias} \\
    \delta{\gyrBias}
  \end{bmatrix}
  \enspace
  \Vec{u} = \begin{bmatrix}
    \accMeas \\
    \gyrMeas
  \end{bmatrix}
  \enspace
  \Vec{\noise} = \begin{bmatrix}
    \accNoise \\
    \gyrNoise \\
    \accBiasNoise \\
    \gyrBiasNoise
  \end{bmatrix} \\

.. math::
  \delta{\dot{\Vec{x}}} &= \Mat{F} \delta{\Vec{x}} + \Mat{G} \Vec{n} \\
  \begin{bmatrix}
    \dotdalpha \\
    \dotdbeta \\
    \dotdtheta \\
    \delta{\dot{\accBias}} \\
    \delta{\dot{\gyrBias}}
  \end{bmatrix}
  &=
  \begin{bmatrix}
    % ROW 1
    \mathbf{0}_{3}
    & \I_{3}
    & \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & \mathbf{0}_{3} \\
    % ROW 2
    \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & -\rot_{{S_{k}S_{t}}} \Skew{\accMeas - \accBias}
    & -\rot_{{S_{k}S_{t}}}
    & \mathbf{0}_{3} \\
    % ROW 3
    \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & -\Skew{\gyrMeas - \gyrBias}
    & \mathbf{0}_{3}
    & -\I_{3} \\
    % ROW 4
    \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & \mathbf{0}_{3} \\
    % ROW 5
    \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & \mathbf{0}_{3}
    & \mathbf{0}_{3} \\
  \end{bmatrix}
  \begin{bmatrix}
    \dalpha \\
    \dbeta \\
    \dtheta \\
    \delta{\accBias} \\
    \delta{\gyrBias}
  \end{bmatrix} \\
  &+
  \begin{bmatrix}
    % ROW 1
    \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} \\
    % ROW 2
    -\rot_{{S_{k}S_{t}}} & \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} \\
    % ROW 3
    \mathbf{0}_{3} & -\I_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} \\
    % ROW 4
    \mathbf{0}_{3} & \mathbf{0}_{3} & \I_{3} & \mathbf{0}_{3} \\
    % ROW 5
    \mathbf{0}_{3} & \mathbf{0}_{3} & \mathbf{0}_{3} & \I_{3}
  \end{bmatrix}
  \begin{bmatrix}
    \accNoise \\
    \gyrNoise \\
    \accBiasNoise \\
    \gyrBiasNoise
  \end{bmatrix} \\


.. math::
  \Mat{P} &=
    (\I + \Mat{F} \delta{t}) \Mat{P} (\I + \Mat{F} \delta{t})^{\transpose}
    + (\Mat{G} \delta{t}) \Mat{Q} (\Mat{G} \delta{t})^{\transpose} \\
  \Mat{J} &=
    (\I + \Mat{F} \delta{t}) \Mat{J}

.. math::
  \boldsymbol{\alpha}_{k,k+1} &\approx
    \underbrace{\hat{\boldsymbol{\alpha}}_{k,k+1}}_{\text{preintegrated}}
    + \Mat{J}^{\alpha}_{\accBias} \delta{\accBias}
    + \Mat{J}^{\alpha}_{\gyrBias} \delta{\gyrBias} \\
  \boldsymbol{\beta}_{k,k+1} &\approx
    \underbrace{\hat{\boldsymbol{\beta}}_{k,k+1}}_{\text{preintegrated}}
    + \Mat{J}^{\beta}_{\accBias} \delta{\accBias}
    + \Mat{J}^{\beta}_{\gyrBias} \delta{\gyrBias} \\
  \boldsymbol{\gamma}_{k,k+1} &\approx
    \underbrace{\hat{\boldsymbol{\gamma}}_{k,k+1}}_{\text{preintegrated}}
    \otimes
    \begin{bmatrix}
      1 \\
      \dfrac{1}{2} \Mat{J}^{\gamma}_{\gyrBias} \delta{\gyrBias}
    \end{bmatrix}

Residuals

.. math::
  \begin{bmatrix}
    \Vec{e}_{\dalpha} \\
    \Vec{e}_{\dbeta} \\
    \Vec{e}_{\dgamma} \\
    \Vec{e}_{\delta{\accBias}} \\
    \Vec{e}_{\delta{\gyrBias}}
  \end{bmatrix}
  =
  \underbrace{
    \begin{bmatrix}
    % Position
        \rot_{{S_{k}W}} \left(
          \pos_{{WS}_{k+1}}
          - \pos_{{WS}_{k}}
          - \vel_{{WS}_{k}} \Delta{t}
          + \dfrac{1}{2} \gravity \Delta{t^{2}}
        \right) \\
    % Velocity
        \rot_{{S_{k}W}} \left(
          \vel_{{WS}_{k+1}}
          - \vel_{{WS}_{k}}
          + \gravity \Delta{t}
        \right) \\
    % Orientation
    \quat^{-1}_{WS_{k}} \otimes \quat_{WS_{k+1}} \\
    % Biases
    \accBias_{k+1} - \accBias_{k} \\
    \gyrBias_{k+1} - \gyrBias_{k}
    \end{bmatrix}
  }_{\text{Measured}}
  -
  \underbrace{
    \begin{bmatrix}
    \hat{\boldsymbol{\alpha}}_{k,k+1} \\
    \hat{\boldsymbol{\beta}}_{k,k+1} \\
    \hat{\boldsymbol{\gamma}}_{k,k+1} \\
    \Mat{0} \\
    \Mat{0}
    \end{bmatrix}
  }_{\text{Predicted}}

Jacobians

Pose at :math:`k`

.. math::
  \begin{align}
    \dfrac{\Vec{e}_{\dalpha}}{{\partial{\pos_{WS_{k}}}}}
      &= -\rot_{S_{k}W} \\
    \dfrac{\Vec{e}_{\dalpha}}{{\partial{\dtheta_{k}}}}
      &=
      \Skew{
        \rot_{{S_{k}W}} \left(
          \pos_{{WS}_{k+1}}
          - \pos_{{WS}_{k}}
          - \vel_{{WS}_{k+1}} \Delta{t}
          + \dfrac{1}{2} \gravity \Delta{t^{2}}
        \right)
      } \\
    \dfrac{\Vec{e}_{\dgamma}}{{\partial{\dtheta_{k}}}} &=\\
    \dfrac{\Vec{e}_{\dbeta}}{{\partial{\pos_{WS_{k}}}}}
      &=
      \Skew{
        \rot_{{S_{k}W}} \left(
          \vel_{{WS}_{k+1}}
          - \vel_{{WS}_{k}}
          + \gravity \Delta{t}
        \right)
      } \\
  \end{align}

Speed and Biases at :math:`k`

.. math::
  \begin{align}
    \dfrac{\partial{\error_{\dalpha}}}{{\partial{\vel_{WS_{k}}}}}
      &= -\rot_{S_{k}W} \Delta{t} \\
    \dfrac{\partial{\error_{\dalpha}}}{{\partial{\accBias_{k}}}}
      &= -\Mat{J}^{\alpha}_{\delta{\accBias}} \\
    \dfrac{\partial{\error_{\dalpha}}}{{\partial{\gyrBias_{k}}}}
      &= -\Mat{J}^{\alpha}_{\delta{\gyrBias}}
  \end{align}

.. math::
  \begin{align}
    \dfrac{\partial{\error_{\dbeta}}}{{\partial{\vel_{WS_{k}}}}}
      &= -\rot_{S_{k}W} \\
    \dfrac{\partial{\error_{\dbeta}}}{{\partial{\accBias_{k}}}}
      &= -\Mat{J}^{\beta}_{\delta{\accBias}} \\
    \dfrac{\partial{\error_{\dbeta}}}{{\partial{\gyrBias_{k}}}}
      &= -\Mat{J}^{\beta}_{\delta{\gyrBias}}
  \end{align}

.. math::
  \begin{align}
    \dfrac{\partial{\error_{\delta{\accBias}}}}{{\partial{\accBias_{k}}}}
      &= -\I_{3} \\
    \dfrac{\partial{\error_{\delta{\gyrBias}}}}{{\partial{\gyrBias_{k}}}}
      &= -\I_{3}
  \end{align}

Pose at :math:`k+1`

.. math::
  \begin{align}
    \dfrac{\partial\Vec{e}_{\dalpha}}{{\partial{\pos_{WS_{k+1}}}}}
      &= \rot_{S_{k}W} \\
    \dfrac{\partial\Vec{e}_{\dgamma}}{{\partial{\quat_{WS_{k+1}}}}}
      &= \\
  \end{align}

Speed and Biases at :math:`k+1`

.. math::
  \begin{align}
    \dfrac{\partial{\error_{\dbeta}}}{{\partial{\vel_{WS_{k+1}}}}}
      &= \rot_{S_{k}W} \\
    \dfrac{\partial{\error_{\delta{\accBias}}}}{{\partial{\accBias_{k+1}}}}
      &= \I_{3} \\
    \dfrac{\partial{\error_{\delta{\gyrBias}}}}{{\partial{\gyrBias_{k+1}}}}
      &= \I_{3}
  \end{align}

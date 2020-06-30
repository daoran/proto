Inertial Measurement Unit (IMU)
===============================

IMU Motion Model
----------------

.. math::
  \begin{align}
    % Position
    \dot\pos_{WS} &= \vel_{WS} \\
    % Orientation
    \dot\quat_{WS} &=
      \dfrac{1}{2} \mathbf{\Omega}
      (\gyrMeas, \gyrNoise, \gyrBias)
      \quat_{WS} \\
    % Velocity
    \dot\vel_{WS} &=
      \rot_{WS}
      (\accMeas + \accNoise - \accBias) + \gravity \\
    % Gyro Bias
    \dot{\gyrBias} &= \noise_{\gyrBias} \\
    % Accel Bias
    \dot{\accBias} &= -\dfrac{1}{\tau} \accBias + \noise_{\gyrBias}
  \end{align}

The matrix :math:`\mathbf{\Omega}` is formed from the estimated angular rate
:math:`\gyr = \gyrMeas + \gyrNoise - \gyrBias`.

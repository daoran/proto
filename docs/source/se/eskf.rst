Error-State Kalman Filter
=========================

True state kinematics
----------------------

.. math::

  \dot\pos &= \vel \\
  \dot\vel &= \acc \\
  \dot\quat &= \dfrac{1}{2} \quat \otimes \angvel \\
  \dot\bias_{\acc} &= \noise_{\bias_{\acc}} \\
  \dot\bias_{\gyr} &= \noise_{\bias_{\gyr}}

True acceleration and angular rate are obtained from an IMU in the form of noisy
sensor readings expresssed in the sensor frame, namely

.. math::

  \acc_{m} &= \rot (\acc - \gravity) + \bias_{\acc} + \noise_{\acc} \\
  \gyr_{m} &= \angvel + \bias_{\gyr} + \noise_{\gyr}

.. math::
  \state = \begin{bmatrix}
    \pos \\
    \vel \\
    \quat \\
    \bias_{\acc} \\
    \bias_{\gyr}
  \end{bmatrix}
  \enspace
  \Vec{u} = \begin{bmatrix}
    \acc_{m} - \noise_{\acc} \\
    \gyr_{m} - \noise_{\gyr}
  \end{bmatrix}
  \enspace
  \noise = \begin{bmatrix}
    \noise_{\bias_{\acc}} \\
    \noise_{\bias_{\gyr}}
  \end{bmatrix}

Nominal state kinematics
------------------------

.. math::

  \dot\pos &= \vel \\
  \dot\vel &= \rot (\acc_{m} - \bias_{\acc}) + \gravity \\
  \dot\quat &= \dfrac{1}{2} \quat \otimes (\gyr_{m} - \bias_{\gyr}) \\
  \dot\bias_{\acc} &= \zeros_{3 \times 1} \\
  \dot\bias_{\gyr} &= \zeros_{3 \times 1}

Error state kinematics
----------------------

.. math::

  \dot{\delta\pos} &= \delta\vel \\
  \dot{\delta\vel} &= -\rot \Skew{\acc_{m} - \bias_{\acc}} \dtheta
    - \rot \delta\bias_{\acc}
    + \delta\gravity
    - \rot \noise_{\acc} \\
  \dot{\dtheta} &= -\Skew{\gyr_{m} - \bias_{\gyr}} \dtheta
    - \delta\bias_{\gyr}
    - \noise_{\gyr} \\
  \dot{\delta\bias_{\acc}} &= \noise_{\bias_{\acc}} \\
  \dot{\delta\bias_{\gyr}} &= \noise_{\bias_{\gyr}}


Error state Jacobian
--------------------

.. math::

  \Mat{F}_{\state}
    = \dfrac{\partial f}{\partial \delta\state}
    = \begin{bmatrix}
      % Row 1
      \zeros_{3\times3}
      & \I_{3}
      & \zeros_{3\times3}
      & \zeros_{3\times3}
      & \zeros_{3\times3}
      \\ % Row 2
      \zeros_{3\times3}
      & \zeros_{3\times3}
      & -\rot \Skew{\acc_{m} - \bias_{\acc}}
      & -\rot
      & \zeros_{3\times3}
      \\ % Row 3
      \zeros_{3\times3}
      & \zeros_{3\times3}
      & -\Skew{\gyr_{m} - \bias_{\gyr}}
      & \zeros_{3\times3}
      & -\I_{3\times3}
      \\ % Row 4
      \zeros_{3\times3}
      & \zeros_{3\times3}
      & \zeros_{3\times3}
      & \zeros_{3\times3}
      & \zeros_{3\times3}
      \\ % Row 4
      \zeros_{3\times3}
      & \zeros_{3\times3}
      & \zeros_{3\times3}
      & \zeros_{3\times3}
      & \zeros_{3\times3}
    \end{bmatrix}



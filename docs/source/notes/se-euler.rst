Euler Angles
============

Z-Y-X rotation sequence:

.. math::
  \rot_{zyx} =
  \begin{bmatrix}
    c(\psi) c(\theta)
    & c(\psi) s(\theta) s(\phi) - s(\psi) c(\phi)
    & c(\psi) s(\theta) c(\phi) + s(\psi) s(\phi) \\
    s(\psi) c(\theta)
    & s(\psi) s(\theta) s(\phi) + c(\psi) c(\phi)
    & s(\psi) s(\theta) c(\phi) - c(\psi) s(\phi) \\
    -s(\theta) & c(\theta) s(\phi) & c(\theta) c(\phi)
  \end{bmatrix}


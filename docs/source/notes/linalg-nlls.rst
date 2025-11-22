Non-linear Least Squares
========================

Gauss Newton
------------

.. math::
  \min_{\Vec{x}} \cost(\Vec{x})
    &=
      \dfrac{1}{2}
      \sum_{i}
      \Vec{e}_{i}^{\transpose} \Mat{W} \Vec{e}_{i} \\
    &=
      \dfrac{1}{2} \enspace
      \Vec{e}_{i}^{\transpose}(\Vec{x})
      \Mat{W}
      \Vec{e}_{i}(\Vec{x})

where the error function, :math:`\Vec{e}(\cdot)`, depends on the optimization
parameter, :math:`\Vec{x} \in \real^{n}`. The error function,
:math:`\Vec{e}(\cdot)`, has a form of

.. math::
  \Vec{e}_{i} =
    \Vec{z} - \Vec{h}(\Vec{x})

is defined as the difference between the measured value, :math:`\Vec{z}`, and
the estimated value calculated using the measurement function,
:math:`\Vec{h}(\cdot)`.  Since the error function, :math:`\Vec{e}(\Vec{x})`, is
non-linear, it is approximated with the first-order Taylor series,

.. math::
  \Vec{e}(\Vec{x})
    \approx
      \Vec{e}(\bar{\Vec{x}}) +
      \Mat{E}(\bar{\Vec{x}}) \Delta\Vec{x}

where :math:`\Mat{E}(\bar{\Vec{x}}) =
\dfrac{\partial\Vec{e}(\Vec{x})}{\partial\Vec{x}} \bigg\rvert_{\Vec{x}_{k}}`
and :math:`\Delta{\Vec{x}} = \Vec{x} - \bar{\Vec{x}}`.

.. math::
  \dfrac{\partial{\cost}}{\partial{\Vec{x}}} =
    \dfrac{\partial{\cost}}{\partial{\Vec{e}}}
    \dfrac{\partial{\Vec{e}}}{\partial{\Vec{x}}}


.. math::

  \dfrac{\partial{\cost}}{\partial{\Vec{e}}} &=
    \dfrac{1}{2} \Vec{e}^{\transpose}(\Vec{x}) \Mat{W} \Vec{e}(\Vec{x}) =
    \Vec{e}^{\transpose}(\Vec{x}) \Mat{W} \\
  %
  \dfrac{\partial{\Vec{e}}}{\partial{\Vec{x}}} &=
    \Vec{e}(\bar{\Vec{x}}) +
    \Mat{E}(\bar{\Vec{x}}) \Delta\Vec{x} =
    \Mat{E}(\bar{\Vec{x}})



.. math::

  \dfrac{\partial{\cost}}{\partial{\Vec{x}}}
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



.. math::
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



Solve the normal equations :math:`\Mat{H}\Delta\Vec{x} = \Vec{b}` for
:math:`\Delta\Vec{x}` using the Cholesky or QR-decompositon. Once
:math:`\Delta\Vec{x}` is found the best estimate :math:`\bar{\Vec{x}}` can be
updated via,

.. math::
  \bar{\Vec{x}}_{k + 1} = \bar{\Vec{x}}_{k} + \Delta\Vec{x}.

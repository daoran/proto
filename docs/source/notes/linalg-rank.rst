Rank
====

The rank :math:`\rho(\Mat{A})` of a matrix :math:`\Mat{A}` of :math:`n` rows
and :math:`m` columns is defined as **the number of independent rows or
columns**. Rank is thus a measure of the "non-degenerateness" of the system of
lienar equations and linear transformation encoded by :math:`\Mat{A}`.

* Full rank (non-singular): :math:`\rho(\Mat{A}) = \min(n, m)`
* Not full rank (singular): :math:`\rho(\Mat{A}) < \min(n, m)`



Rank Properties
---------------

Let :math:`\Mat{A}` be an :math:`m \times n` matrix, and :math:`\Mat{B}` be an
:math:`n \times k` matrix,

.. math::

  \Rank{\Mat{A}} &\leq \Min{m}{n} \\
  \Rank{\Mat{AB}} &\leq \Min{\Rank{\Mat{A}}}{\Rank{\Mat{B}}} \\
  \Rank{\Mat{AB}} + \Rank{\Mat{BC}}
    &\leq \Rank{\Mat{B}} + \Rank{\Mat{ABC}} \\
  \Rank{\Mat{A} + \Mat{B}} &\leq \Rank{\Mat{A}} + \Rank{\Mat{B}} \\
  \Rank{\Mat{A}^{\transpose} \Mat{A}}
    &= \Rank{\Mat{A} \Mat{A}^{\transpose}}
    = \Rank{\Mat{A}}
    = \Rank{\Mat{A}^{\transpose}} \\
  \Rank{\Mat{0}} &= 0

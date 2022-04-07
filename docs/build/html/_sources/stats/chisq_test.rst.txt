Pearson's Chi-Squared Test
==========================

Pearson's chi-squared test :math:`\chi^2` is a statistical test applied to sets
of *categorial data* to quantify the likelihood that the observed difference
between measured and predicted arose by chance. The test is used to assess
three types of comparisons:

* **Goodness of fit**: checks whether the observed frequency distribution
  matches the theoretical distribution.
* **Homogeneity**: compares the distribution of counts for two or more groups
  using the same categorical variable.
* **Independence**: checks whether the unparied observations on two variables,
  expressed in a contingency table, are independent of each other.



Goodness of Fit
---------------

.. math::

  \chi^2 &= \sum^{n}_{i=1} \dfrac{(O_i - E_i)^2}{E_i} \\
         &= N \sum^{n}_{i=1} \dfrac{(O_i / N - p_i)^2}{p_i}

where :math:`\chi^2` is Pearson's cumulative test statistic, :math:`O_i` is the
number of observations of type :math:`i`, :math:`N` is the total number of
observations, :math:`E_i = N p_i` is the expected (theoretical) count of type
:math:`i`, and :math:`n` is the number of cells in the table.

Once the chi-squared test statistic is calculated, the :math:`p`-value is
obtained by comparing the value of the statistic to a chi-squared distribution.
The number of degrees of freedom is equal to the number of cells :math:`n`,
minus the reduction in degrees of freedom, :math:`p`.


Example: Fairness of dice
^^^^^^^^^^^^^^^^^^^^^^^^^

A 6-sided die is thrown 60 times. The number of times it lands with 1, 2, 3, 4,
5 and 6 face up is 5, 8, 9, 8, 20 and 20, respectively. Is the die biased,
according to the Pearson's chi-squared test at a significance level of 95%,
and, or 99%?

In this example :math:`n = 6` as there are 6 possible outcomes, 1 to 6. The
null hypothesis is that the die is unbaised, hence each dice number is expected
to occur the same number of times, in this case, :math:`60 / n = 10`. The
outcomes can be tabulated as follows:

+----------+-------------+-------------+-------------------+-----------------------+-----------------------------------+
|:math:`i` | :math:`O_i` | :math:`E_i` | :math:`O_i - E_i` | :math:`(O_i - E_i)^2` | :math:`\dfrac{(O_i - E_i)^2}{E_i}`|
+----------+-------------+-------------+-------------------+-----------------------+-----------------------------------+
| 1        + 5           + 10          + 5                 + 25                    + 2.5                               |
+----------+-------------+-------------+-------------------+-----------------------+-----------------------------------+
| 2        | 8           | 10          | 2                 | 4                     | 0.4                               |
+----------+-------------+-------------+-------------------+-----------------------+-----------------------------------+
| 3        | 9           | 10          | 1                 | 1                     | 0.1                               |
+----------+-------------+-------------+-------------------+-----------------------+-----------------------------------+
| 4        | 8           | 10          | 2                 | 4                     | 0.4                               |
+----------+-------------+-------------+-------------------+-----------------------+-----------------------------------+
| 5        | 10          | 10          | 0                 | 0                     | 0                                 |
+----------+-------------+-------------+-------------------+-----------------------+-----------------------------------+
| 6        | 20          | 10          | 10                | 100                   | 10                                |
+----------+-------------+-------------+-------------------+-----------------------+-----------------------------------+

The number of degrees of freedom is :math:`n - 1 = 5`. The Upper tail critical
values of chi-square distribution gives a critical value of 11.070 at 95%
significance level:

+-----+------------------------------------------+--------+--------+---------+--------+----------+
| Dof | Probability less than the critical value | *0.90* | *0.95* | *0.975* | *0.99* | *0.999*  |
+-----+------------------------------------------+--------+--------+---------+--------+----------+
| 5   |                                          | 9.236  | 11.070 | 12.833  | 15.086 | 20.515   |
+-----+------------------------------------------+--------+--------+---------+--------+----------+

As the chi-squared statistic of 13.4 exceeds this critical value, the null
hypothesis is rejected and conclude that the die is biased at 95% significance
level. At 99% significance level, the critical value is 15.086. As the
chi-squared statistic does not exceed it, the null hypothesis is not rejected
and thus conclude that there is insufficient evidence to show that the die is
biased at 99% significance level.


Problems with Pearson's Chi-Squared Test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* The degrees of freedom can only be estimated for *linear
  models*. It is non-trivial or near impossible to estimate the degrees of
  freedom for a *non-linear model*.

* The value of chi-squared itself is subject to noise in the data, as
  such the value is uncertain.

Knowing the degrees of freedom of the model in question is required for
chi-squared test. For :math:`N` data points and :math:`P` parameters, a naive
guess is that the number of degrees of freedom is :math:`N - P`. This, however,
as will be demonstrated is not always the case.

The chi-squared test statistic, :math:`\chi^2`, for continuous data is defined
as,

.. math::

  \chi^2 = \sum^{N}_{n = 1}
    \left( \dfrac{(y_n - f(x_n, \theta))}{\sigma_n} \right)^2

which is equivalent to maximizing the liklihood function. For a linear model,
the chi-squared statistic, :math:`\chi^2` can be written in matrix form as,

.. math::

  \chi^2 =
    (\Vec{y} - \Mat{X} \boldsymbol{\theta})^{\transpose}
    \Mat{\Sigma}^{-1}
    (\Vec{y} - \Mat{X} \boldsymbol{\theta}) .

Rearranging in terms of the model parameters, :math:`\boldsymbol{\theta}`,
gives,

.. math::

  \boldsymbol{\theta} =
    (\Mat{X}^{\transpose} \Mat{\Sigma}^{-1} \Mat{X})^{-1}
    \Mat{X}^{\transpose} \Mat{\Sigma}^{-1} \Vec{y} .

Finally, the prediction, :math:`\hat{\Vec{y}}`, of the measurements,
:math:`\Vec{y}`, can be obtained by multiplying the design matrix,
:math:`\Mat{X}`, with the model parameters, :math:`\boldsymbol{\theta}`, and
further simplified to give us,

.. math::

  \begin{align}
    \hat{\Vec{y}}
      &= \Mat{X} \boldsymbol{\theta} \\
      &= \Mat{X} (\Transpose{\Mat{X}} \Mat{\Sigma}^{-1} \Mat{X})^{-1}
         \Transpose{\Mat{X}} \Mat{\Sigma}^{-1} \Vec{y} \\
      &= \Mat{H} \Vec{y}
  \end{align}

where :math:`\Mat{H}` is an `N \times N` matrix, sometimes called the "hat
matrix" because it translates the measurement data, :math:`\Vec{y}`, into a
model prediction, :math:`\hat{\Vec{y}}`. The number of *effecitve* model
parameters, :math:`{P}_{\text{eff}}`, is then given by the trace of
:math:`\Mat{H}`,

.. math::

  P_{\text{eff}} = \Trace{\Mat{H}} = \sum^{N}_{n = 1} H_{nn} = \Rank{\Mat{X}} .

which also equals the rank of the design matrix :math:`\Mat{X}`. And so,
:math:`P_{\text{eff}} \leq P`, where the equality holds if and only if the
design matrix :math:`\Mat{X}` has full rank. Consequently, for linear models
the number of degrees of freedom is,

.. math::

  K = N - P_{\text{eff}} \geq N - P

For nonlinear models, the degrees of freedom is not as straight forward.


**Example 1**

Let us consider a nonlinear model with three free parameters, :math:`A`,
:math:`B`, and :math:`C`,

.. math::

  f(x) = A \cos(Bx + C) .

If we are given a set of :math:`N` measurement :math:`(x_n, y_n, \sigma_n)`
such that no two data points have identical :math:`x_n`, then the model
:math:`f(x)` is capable of fitting any such data set perfectly. The way this
works is by increasing the "frequency" :math:`B` such that :math:`f(x)` can
change on arbitrarily short scales. As :math:`f(x)` provides a perfect fit in
this case, :math:`\chi^2` is equal to zero for all possible noise realizations
of the data. Evidently, this three-parameter model has infinite flexibility (if
there are no priors) and :math:`K = N - P` is a poor estimate of the number of
degrees of freedom, which actually is :math:`K = 0`.


**Example 2**

To build upon the first example, three additional model parameters, `D`, `E`
and `F` are added,

.. math::

  f(x) = A \cos(Bx + C) + D \cos(Ex + F) .

If the fit parameter :math:`D` becomes small such that :math:`|D| \leq |A|`,
the second component cannot influence the fit anymore and the two model
parameters :math:`E` and :math:`F` are "lost". In simple words: This model may
change its flexibility during the fitting procedure.

Hence, for nonlinear models, :math:`K` may not even be constant. Of course,
these two examples do not verify the claim that always :math:`K \neq N - P` for
nonlinear models.  However, acting as counter-examples, they clearly falsify
the claim that :math:`K = N - P` is always true for nonlinear models.

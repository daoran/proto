Variance
========

Variance is a measure of spread, it is the average squared distance between
measurements and the mean.

The **Population variance** is defined as,

.. math::

  \sigma^{2} = \dfrac{\sum_{i=1}^{N} (x_i - \mu)^{2}}{N}.


and the **Sample variance** is defined as,

.. math::

  \sigma^{2} = \dfrac{\sum_{i=1}^{n} (x_i - \mu)^{2}}{n - 1}.

Where :math:`N` is the population size and :math:`n` is the sample size.

In plain english, the population variance, standard deviation, etc, is when the
whole population data is available. Sample variance, standard deviation, etc,
on the other hand implies the availble data is only a subset of the population.

The problem with sample data is that estimates without applying the :math:`n -
1` correction will often underestimate the true value. This is particularly
true if :math:`n` is small. By using :math:`n - 1` instead of :math:`n` as the
divsor corrects the estimate by making the result slightly larger.

The degrees of freedom is another perspective of why :math:`n - 1` should be
use when estimating from sample data. In statistics the degrees of freedom
describe the number variables that could affect the response or output of a
model. (need more explaining)

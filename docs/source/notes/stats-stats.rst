Statistics Reference
====================

Mean
----
.. math::

  \mu = \dfrac{1}{N} \sum_{i=1}^{N} x_{i}


Variance
--------

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


Standard Deviation
------------------

The standard deviation :math:`\sigma` is defined as the square root of the
variance :math:`\sigma^{2}`. It is also a measure of spread in the data.
However standard deviation is often more intuitive, because instead of the
spread in terms of distance from the \textit{mean squared}, the standard
deviation is simply the distance from the mean (not *mean squared*).

.. math::

  \sigma = \sqrt{\sigma^2}


Standard Error
--------------

Standard error of the regression, or standard error of the estimate is the
average distance between observed values and the fitted model. This metric
conveys objectively how wrong the regression model is on average using the
units of the response variable. A small standard error indicates observations
are closer to the fitted model. Conceptually it is similar to the standard
deviation, the difference is standard deviation measures te average distance of
the observed alues from the mean.

The standard error (:math:`S`) is defined as:

.. math::

  S = \sqrt{\dfrac{\sum_{i=1}^{n} (\hat{y}_{i} - y_{i})^{2}}{df}}


Bayes Theorem
-------------

.. math::

  p(\RV{x}, \RV{y}) = p(\RV{x}|\RV{y}) p(\RV{y}) = p(\RV{x}|\RV{y}) p(\RV{y})


.. math::

  \underbrace{p(\RV{x}|\RV{y})}_{\text{Posterior}} =
    \dfrac{
      \overbrace{p(\RV{x}|\RV{y})}^{\text{Likelihood}}
      \enspace
      \overbrace{p(\RV{y})}^{\text{Prior}}
    }{
      \underbrace{p(\RV{y})}_{\text{Evidence}}
    }

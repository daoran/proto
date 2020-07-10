Standard Error
==============

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

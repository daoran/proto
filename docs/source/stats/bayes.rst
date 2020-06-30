Bayes Theorem
=============

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

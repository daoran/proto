Shannon Information
===================

.. math::

  \begin{align}
    H(\mathbf{x})
      &= - \E{\ln p(\mathbf{\Vec{x}})} \\
      &= - \int p(\Vec{x}) \ln p(\Vec{x}) dx \\
      &= - \int_{-\infty}^{\infty} p(\Vec{x}) \ln \left(
        \frac{1}{\sqrt{(2 \pi)^{N} \text{det}({\mathbf{\Sigma}})}}
        \exp\left(
          -\dfrac{1}{2} (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          \Sigma^{-1} (\Vec{x} - \boldsymbol{\mu})
        \right)
      \right) \\
      &= - \int_{-\infty}^{\infty} p(\Vec{x}) \left(
        - \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
          -\dfrac{1}{2} (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          \Sigma^{-1} (\Vec{x} - \boldsymbol{\mu})
      \right) \\
      &= \int_{-\infty}^{\infty}
        \frac{1}{2} \ln (2 \pi)^{N} 
        \text{det}({\mathbf{\Sigma}}) \enspace p(\Vec{x})
        + \int_{-\infty}^{\infty}
        \dfrac{1}{2} (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          \Sigma^{-1} (\Vec{x} - \boldsymbol{\mu})
          \enspace p(\Vec{x}) \\
      &= \int_{-\infty}^{\infty}
        \frac{1}{2} \ln (2 \pi)^{N} 
        \text{det}({\mathbf{\Sigma}}) \enspace p(\Vec{x})
        + \int_{-\infty}^{\infty}
        \dfrac{1}{2} (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          \Sigma^{-1} (\Vec{x} - \boldsymbol{\mu})
          \enspace p(\Vec{x}) \\
      &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
          \int_{-\infty}^{\infty} p(\Vec{x})
        + \int_{-\infty}^{\infty}
        \dfrac{1}{2} (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          \Sigma^{-1} (\Vec{x} - \boldsymbol{\mu})
          \enspace p(\Vec{x}) \\
      &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
        + \int_{-\infty}^{\infty}
        \dfrac{1}{2} (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          \Sigma^{-1} (\Vec{x} - \boldsymbol{\mu})
          \enspace p(\Vec{x}) \\
      &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
        + \E{
          \dfrac{1}{2} (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          \Sigma^{-1} (\Vec{x} - \boldsymbol{\mu})
        }
  \end{align}

.. math::

  \begin{align}
    % Line 1
    (\Vec{x} - \boldsymbol{\mu})^{\transpose}
    \mathbf{\Sigma}^{-1}
    (\Vec{x} - \boldsymbol{\mu})
    &=
      \Trace{
        \mathbf{\Sigma}^{-1}
        (\Vec{x} - \boldsymbol{\mu})
        (\Vec{x} - \boldsymbol{\mu})^{\transpose}
      } \\
  \end{align}

.. math::

  \begin{align}
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} \E{
        (\Vec{x} - \boldsymbol{\mu})^{\transpose}
        \Sigma^{-1} (\Vec{x} - \boldsymbol{\mu})
      } \\
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} \E{
          \Trace{
            \mathbf{\Sigma}^{-1}
            (\Vec{x} - \boldsymbol{\mu})
            (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          }
      } \\
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} \Trace{
          \E{
            \mathbf{\Sigma}^{-1}
            (\Vec{x} - \boldsymbol{\mu})
            (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          }
      } \\
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} \Trace{
          \mathbf{\Sigma}^{-1}
          \E{
            (\Vec{x} - \boldsymbol{\mu})
            (\Vec{x} - \boldsymbol{\mu})^{\transpose}
          }
      } \\
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} \Trace{
          \mathbf{\Sigma}^{-1}
          \mathbf{\Sigma}
      } \\
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} \Trace{
        \I
      } \\
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} N \\
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} N \ln e \\
    &= \frac{1}{2} \ln (2 \pi)^{N} \text{det}({\mathbf{\Sigma}})
      + \dfrac{1}{2} \ln e^{N} \\
  \end{align}

.. math::
  \begin{align}
    H(\Vec{x}) = \frac{1}{2} \ln (2 \pi e)^{N} \text{det}({\mathbf{\Sigma}})
  \end{align}

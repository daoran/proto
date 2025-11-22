Illumination Invariant Transform
================================

The illumination invariant transform takes three input channels from the
image, and returns a single illumination adjusted channel, :math:`I`, as follows,

.. math::

  I = \log(R_{2}) - \alpha \log(R_{1}) - (1 - \alpha) \log(R_{3})

where :math:`R_{1}, R_{2}, R_{3}` are sensor responses (or image channels)
corresponding to peak sensitivities at ordered wavelengths :math:`\lambda_{1} <
\lambda_{2} < \lambda_{3}`, and :math:`\alpha` is determined by the following
equations,

.. math::
  :label: illum_invar

  \begin{align}
    \dfrac{1}{\lambda_{2}} &=
      \dfrac{\alpha}{\lambda_{1}}
      + \dfrac{\left(1 - \alpha \right)}{\lambda_{3}} \\
      \alpha &= \dfrac{\lambda_{1} (\lambda_{2} - \lambda_{3})}
      {\lambda_{2} (\lambda_{1} - \lambda_{3})}
  \end{align}


This transform, however, has a non-intuitive effect on black and white targets,
as the three channels tend to be equally over and under exposed in RGB images.
As a result, the transform leads to similar values for white and black pixels,
eliminating the ability for the computer vision algorithms to detect edges.


[Maddern2014]: Maddern, Will, et al. "Illumination invariant imaging:
Applications in robust vision-based localisation, mapping and classification
for autonomous vehicles." Proceedings of the Visual Place Recognition in
Changing Environments Workshop, IEEE International Conference on Robotics and
Automation (ICRA), Hong Kong, China. Vol. 2. 2014.

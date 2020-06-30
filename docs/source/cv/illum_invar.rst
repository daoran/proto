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



Illumination Invariant AprilTags
--------------------------------

Robust AprilTag detection emerged from experience with the standard
black and white AprilTag during outdoor experiments, where the detection
becomes unreliable in certain lighting conditions. In particular, detection
fails when strong shadows cover tag features fully or partially. The cause of
failure is due to how the detection process relies on image gradients to detect
the edges and lines of the tag in order to extract the relative tag pose.
Depending on the time of day or weather conditions, this can have a significant
impact on reliable AprilTag detection. This sensitivity to illumination was
addressed by using the illumination invariant transform by [Maddern2014].

.. figure:: imgs/illum_invar_apriltags_compare.png

To resolve this issue, we designed a new AprilTag so that the single channel
image produced by using the equation introduced above produces a grey scale
like image that is robust to shadows and changes in illumination.  Examining
:eq:`illum_invar`, it can be observed the resulting pixel intensities are
maximized when the camera observes green (:math:`R_{2}`) and minimized when
viewing a mixture of red and blue, (:math:`R_{1}` and :math:`R_{3}`
respectively). The proposed illumination invariant AprilTag shown in is created
by replacing the white and black portions of a typical AprilTag with green and
magenta. This modification was tested under various lighting conditions.
The figure above shows the tag's appearance after performing the illumination
invariant transform, creating a single channel image that replaces the typical
single channel, grey scale image that is typically used by the AprilTag
library.

The images shown in the figure above are taken using a PointGrey Chameleon3
(CM3-U3-28S4C-CS) with a Sony ICX818 image sensor. The corresponding values of
:math:`\lambda_{1},\lambda_{2}, \lambda_{3}` and :math:`\alpha` are 480 nm,
510 nm, 640 nm and 0.56 respectively as noted in the sensor data sheets.


References
^^^^^^^^^^

[Maddern2014]: Maddern, Will, et al. "Illumination invariant imaging:
Applications in robust vision-based localisation, mapping and classification
for autonomous vehicles." Proceedings of the Visual Place Recognition in
Changing Environments Workshop, IEEE International Conference on Robotics and
Automation (ICRA), Hong Kong, China. Vol. 2. 2014.

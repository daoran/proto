Notations
=========

Robotics is about developing machines that perceive and interact with the
environment. For that we need to represent the state of the robot, its sensors
and its surroundings. The following notes will be using the frame notation as
described by Paul Furgale. The aim is to mitigate the ambiguity that arises
when describing robot poses, sensor data and more.

A vector expressed in the world frame, :math:`\frame_{W}`, is written as
:math:`\pos_{W}`. Or more precisely if the vector describes the position of the
camera frame, :math:`\frame_{C}`, expressed in :math:`\frame_{W}`, the vector
can be written as :math:`\pos_{WC}`. The left hand subscripts indicates the
coordinate system the vector is expressed in, while the right-hand subscripts
indicate the start and end points. For brevity if the vector has the same start
point as the frame to which it is expressed in, the same vector can be written
as :math:`\pos_{WC}`. Similarly a transformation of a point from
:math:`\frame_{W}` to :math:`\frame_{C}` can be represented by a homogeneous
transform matrix, :math:`\tf_{WC}`, where its rotation matrix component is
written as :math:`\rot_{WC}` and the translation component written as
:math:`\trans_{WC}`. A rotation matrix that is parametrized by quaternion
:math:`\quat_{WC}` is written as :math:`\rot\{\quat_{WC}\}`.

.. math::

  &\text{Position:} \enspace & \pos_{WB} \\
  &\text{Velocity:} \enspace & \vel_{WB} \\
  &\text{Acceleration:} \enspace & \acc_{WB} \\
  &\text{Angular velocity:} \enspace & \angvel_{WB} \\
  &\text{Rotation:} \enspace & \rot_{WB} \\
  &\text{Transform:} \enspace & \tf_{WB} \\


.. figure:: imgs/se-frame_dia_example.png
  :align: center

.. figure:: imgs/se-notation_translation.png
  :align: center

.. figure:: imgs/se-notation_angular_velocity.png
  :align: center

.. figure:: imgs/se-notation_rotation.png
  :align: center

.. figure:: imgs/se-notation_transform.png
  :align: center

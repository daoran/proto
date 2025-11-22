Quaternions
===========

A quaternion, :math:`\Vec{q} \in \real^{4}`, generally has the following form

.. math::
  \quat = q_{w} + q_{x} \mathbf{i} + q_{y} \mathbf{j} + q_{z} \mathbf{k},

where :math:`\{ q_{w}, q_{x}, q_{y}, q_{z} \} \in \real` and :math:`\{
\mathbf{i}, \mathbf{j}, \mathbf{k} \}` are the imaginary numbers satisfying

.. math::
  \begin{align}
    &\mathbf{i}^{2}
    = \mathbf{j}^{2}
    = \mathbf{k}^{2}
    = \mathbf{ijk}
    = -1 \\
    \mathbf{ij} = -\mathbf{ji} &= \mathbf{k}, \enspace
    \mathbf{jk} = -\mathbf{kj} = \mathbf{i}, \enspace
    \mathbf{ki} = -\mathbf{ik} = \mathbf{j}
  \end{align}

corresponding to the Hamiltonian convention. The quaternion can be written as a
4 element vector consisting of a \textit{real} (\textit{scalar}) part,
:math:`q_{w}`, and \textit{imaginary} (\textit{vector}) part :math:`\quat_{v}`
as,

.. math::
  \quat =
  \begin{bmatrix} q_{w} \\ \quat_{v} \end{bmatrix} =
  \begin{bmatrix} q_{w} \\ q_{x} \\ q_{y} \\ q_{z} \end{bmatrix}

There are other quaternion conventions, for example, the JPL convention. A more
detailed discussion between Hamiltonian and JPL quaternion convention is
discussed in \cite{Sola2017}.


Main Quaternion Properties
--------------------------

Sum
^^^

Let :math:`\Vec{p}` and :math:`\Vec{q}` be two quaternions, the sum of both
quaternions is,

.. math::
  \Vec{p} \pm \Vec{q} =
  \begin{bmatrix} p_w \\ \Vec{p}_{v} \end{bmatrix}
  \pm
  \begin{bmatrix} q_w \\ \Vec{q}_{v} \end{bmatrix} =
  \begin{bmatrix} p_w \pm q_w \\ \Vec{p}_{v} \pm \Vec{q}_{v} \end{bmatrix}.

The sum between two quaternions :math:`\Vec{p}` and :math:`\Vec{q}` is
**commutative** and **associative**.

.. math::
  \Vec{p} + \Vec{q} = \Vec{q} + \Vec{p}

.. math::
  \Vec{p} + (\Vec{q} + \Vec{r}) = (\Vec{p} + \Vec{q}) + \Vec{r}


Product
^^^^^^^

The quaternion multiplication (or product) of two quaternions :math:`\Vec{p}`
and :math:`\Vec{q}`, denoted by :math:`\otimes` is defined as

.. math::
  \begin{align}
    \Vec{p} \otimes \Vec{q}
      &=
      (p_w + p_x \mathbf{i} + p_y \mathbf{j} + p_z \mathbf{k})
      (q_w + q_x \mathbf{i} + q_y \mathbf{j} + q_z \mathbf{k}) \\
      &=
      \begin{matrix}
        &(p_w q_w - p_x q_x - p_y q_y - p_z q_z)& \\
        &(p_w q_x + p_x q_w + p_y q_z - p_z q_y)& \mathbf{i}\\
        &(p_w q_y - p_y q_w + p_z q_x + p_x q_z)& \mathbf{j}\\
        &(p_w q_z + p_z q_w - p_x q_y + p_y q_x)& \mathbf{k}\\
      \end{matrix} \\
      &=
      \begin{bmatrix}
        p_w q_w - p_x q_x - p_y q_y - p_z q_z \\
        p_w q_x + p_x q_w + p_y q_z - p_z q_y \\
        p_w q_y - p_y q_w + p_z q_x + p_x q_z \\
        p_w q_z + p_z q_w - p_x q_y + p_y q_x \\
      \end{bmatrix} \\
      &=
      \begin{bmatrix}
        p_w q_w - \Transpose{\Vec{p}_{v}} \Vec{q}_{v} \\
        p_w \Vec{q}_{v} + q_w \Vec{p}_{v} + \Vec{p}_{v} \times \Vec{q}_{v}
      \end{bmatrix}.
  \end{align}

The quaternion product is **not commutative** in the general case. There are
exceptions to the general non-commutative rule, where either :math:`\Vec{p}` or
:math:`\Vec{q}` is real such that :math:`\Vec{p}_{v} \times \Vec{q}_{v} = 0`,
or when both :math:`\Vec{p}_v` and :math:`\Vec{q}_v` are parallel,
:math:`\Vec{p}_v || \Vec{q}_v`. Only in these cirmcumstances is the quaternion
product commutative.,

.. math::

  {\Vec{p} \otimes \Vec{q} \neq \Vec{q} \otimes \Vec{p}} \enspace .


The quaternion product is however **associative**,

.. math::

  \Vec{p} \otimes (\Vec{q} \otimes \Vec{r})
  = (\Vec{p} \otimes \Vec{q}) \otimes \Vec{r}


and **distributive over the sum**

.. math::

  \Vec{p} \otimes (\Vec{q} + \Vec{r}) =
  \Vec{p} \otimes \Vec{q} + \Vec{p} \otimes \Vec{r}
  \quad \text{and} \quad
  (\Vec{p} \otimes \Vec{q}) + \Vec{r} =
  \Vec{p} \otimes \Vec{r} + \Vec{q} \otimes \Vec{r}


The quaternion product can alternatively be expressed in matrix form as

.. math::

  \Vec{p} \otimes \Vec{q} = [\Vec{p}]_{L} \Vec{q}
  \quad \text{and} \quad
  \Vec{p} \otimes \Vec{q} = [\Vec{q}]_{R} \Vec{p} \enspace ,


where :math:`[\Vec{p}]_{L}` and :math:`[\Vec{q}]_{R}` are the left and right
quaternion-product matrices which are derived from
\eqref{eq:quaternion_product},

.. math::
  [\Vec{p}]_{L} =
  \begin{bmatrix}
    p_w & -p_x & -p_y & -p_z \\
    p_x & p_w & -p_z & p_y \\
    p_y & p_z & p_w & -p_x \\
    p_z & -p_y & p_x & p_w
  \end{bmatrix},
  \quad \text{and} \quad
  [\Vec{q}]_{R} =
  \begin{bmatrix}
    q_w & -q_x & -q_y & -q_z \\
    q_x & q_w & q_z & -q_y \\
    q_y & -q_z & q_w & q_x \\
    q_z & q_y & -q_x & q_w
  \end{bmatrix},

or inspecting a compact form can be derived as,

.. math::
  [\Vec{p}]_{L} =
  \begin{bmatrix}
    0 & -\Transpose{\Vec{p}_{v}} \\
    \Vec{p}_w \I_{3 \times 3} + \Vec{p}_{v} &
    \Vec{p}_w \I_{3 \times 3} -\Skew{\Vec{p}_{v}}
  \end{bmatrix}

and

.. math::
  [\Vec{q}]_{R} =
  \begin{bmatrix}
    0 & -\Transpose{\Vec{q}_{v}} \\
    \Vec{q}_w \I_{3 \times 3} + \Vec{q}_{v} &
    \Vec{q}_w \I_{3 \times 3} -\Skew{\Vec{q}_{v}}
  \end{bmatrix},

where :math:`\Skew{\bullet}` is the skew operator that produces a matrix cross
product matrix, and is defined as,

.. math::
  \Skew{\Vec{v}} =
  \begin{bmatrix}
    0 & -v_{3} & v_{2} \\
    v_{3} & 0 & -v_{1} \\
    -v_{2} & v_{1} & 0
  \end{bmatrix},
  \quad
  \Vec{v} \in \Real{3}



Conjugate
^^^^^^^^^

The conjugate operator for quaternion, :math:`{(\bullet)}^{\ast}`, is defined
as

.. math::
  \quat^{\ast}
  =
  \begin{bmatrix}
    q_w \\
    - \Vec{q}_v
  \end{bmatrix}
  =
  \begin{bmatrix}
    q_w \\
    - q_x \\
    - q_y \\
    - q_z
  \end{bmatrix}.

This has the properties

.. math::
  \quat \otimes \quat^{-1}
  = \quat^{-1} \otimes \quat
  = q_{w}^{2} + q_{x}^{2} + q_{y}^{2} + q_{z}^{2}
  =
  \begin{bmatrix}
    q_{w}^{2} + q_{x}^{2} + q_{y}^{2} + q_{z}^{2} \\
    \Vec{0}
  \end{bmatrix},

and

.. math::
  (\Vec{p} \otimes \Vec{q})^{\ast}
  = \Vec{q}^{\ast} \otimes \Vec{p}^{\ast}.



Norm
^^^^

The norm of a quaternion is defined by

.. math::
  \begin{align}
   \Norm{\quat} &= \sqrt{\quat \otimes \quat^{\ast}} \\
     &= \sqrt{\quat^{\ast} \otimes \quat} \\
     &= \sqrt{q_{w}^{2} + q_{x}^{2} + q_{y}^{2} + q_{z}^{2}}
     \enspace \in \real,
  \end{align}

and has the property

.. math::
  \Norm{\Vec{p} \otimes \Vec{q}} =
  \Norm{\Vec{q} \otimes \Vec{p}} =
  \Norm{\Vec{p}} \Norm{\Vec{q}}



Quaternion from Two Vectors
---------------------------

TODO: Need to reword the beginning.
Using the properties of the cross and dot product

.. math::
  \Vec{u} \cdot \Vec{v} &=
    \Norm{\Vec{u}} \Norm{\Vec{v}} \cos \theta \\
  \Norm{\Vec{u} \times \Vec{v}} &=
    \Norm{\Vec{u}} \Norm{\Vec{v}} \Norm{\sin \theta} ,

the axis angle, :math:`\boldsymbol{\theta} \in \Real{3}`, can be obtained from
:math:`\Vec{u}` and :math:`\Vec{v}` with

.. math::
  :label: axis_angle

  \begin{align}
    %-- Axis-angle
    \boldsymbol{\theta} &= \theta \Vec{e} \\
    % -- Angle
    \theta &= \cos^{-1}(
      \dfrac{\Vec{u} \cdot \Vec{v}}
            {\Norm{\Vec{u}} \Norm{\Vec{v}}}
    ) \quad , \enspace \theta \in \real \\
    % -- Axis
    \Vec{e} &=
      \dfrac{\Vec{u} \times \Vec{v}}{\Norm{\Vec{u} \times \Vec{v}}}
      \quad , \enspace \Vec{e} \in \Real{3}
  \end{align}


where :math:`\Vec{e}` is the unit vector that defines the rotation axis and :math:`\theta`
is the rotation angle about :math:`\Vec{e}`. Once the axis angle,
:math:`\boldsymbol{\theta}`, is obtained a quaternion can be formed

.. math::
  :label: axis_angle_to_quaternion

  \quat =
    \cos \dfrac{\theta}{2}
    + \Vec{i} \sin \dfrac{\theta}{2} e_{x}
    + \Vec{j} \sin \dfrac{\theta}{2} e_{y}
    + \Vec{k} \sin \dfrac{\theta}{2} e_{z}


Example: Attitude from gravity and accelerometer vectors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In robotics knowing the attitude of the system is often required. An Inertial
Measurement Unit (IMU) is commonly used to obtain this information. Using the
method described previously, a gravity vector along with an accelerometer
measurement vector can be used to obtain an attitude in form of a quaternion.

Let :math:`\Vec{g} \in \Real{3}` be the gravity vector, and :math:`\Vec{a}_{m}
\in \Real{3}` be the accelerometer measurement from an IMU. With the two
vectors :math:`\Vec{g}` and :math:`\Vec{a}_{m}` a quaternion :math:`\quat_{WS}`
expressing the rotation of the IMU sensor frame, :math:`\frame_{S}`, with
respect to the world frame, :math:`\frame_{W}`, can be calculated given that
values for :math:`\Vec{g}` and :math:`\Vec{a}_{m}` are known. For example let

.. math::
  \begin{align}
    % -- Gravity vector
    \Vec{g} &= \Transpose{\begin{bmatrix} 0 & 0 & -9.81 \end{bmatrix}} \\
    % -- Accelerometer measurement vector
    \Vec{a}_{m} &= \Transpose{
      \begin{bmatrix}
        9.2681 &
        -0.310816 &
        -3.14984
      \end{bmatrix}
    } ,
  \end{align}

taken from the first measurement of the `imu_april` calibration sequence of the
EuRoC MAV dataset.

Before calculating the axis-angle, however, it should be noted that when an
accelerometer is at rest the measurement reading in the z-axis is positive
instead of negative. The reason is accelerometers measures acceleration by
measuring the displacement of a proof mass that is suspended with springs. For
example, if gravity is ignored and the accelerometer moves upwards, the proof
mass will be displaced towards the bottom of the accelerometer. This is
interpreted as an acceleration in the upwards direction, and so when the
accelerometer is at rest on a flat surface, gravity pulls on the proof mass
yeilding a positive measurement in the upwards direction. To resolve this issue
the gravity vector is negated, and so :math:`\Vec{u} = -\Vec{g}` and
:math:`\Vec{v} = \Vec{a}_{m}`. Using :eq:`axis_angle` the axis-angle obtained
is:

.. math::
  \begin{align}
    % -- Axis-Angle
    \theta &= 1.8982 \\
    \Vec{e} &= \Transpose{
      \begin{bmatrix}
        0.03352 &
        0.99944 &
        0.00000
      \end{bmatrix}
    }
  \end{align}

Finally the quaternion, :math:`\quat_{WS}`, can be calculated using
:eq:`axis_angle_to_quaternion` resulting in

.. math::
  \quat_{WS} = \Transpose{
    \begin{bmatrix}
      0.58240 &
      0.02725 &
      0.81245 &
      0.00000
    \end{bmatrix}
  } \enspace .



Quaternion to Rotation Matrix
-----------------------------

.. math::
  \rot\{\quat \} = \begin{bmatrix}
    q_w^2 + q_x^2 - q_y^2 - q_z^2
    & 2(q_x q_y - q_w q_z)
    & 2(q_x q_z + q_w q_y) \\
    2(q_x q_y + q_w q_z)
    & q_w^2 - q_x^2 + q_y^2 - q_z^2
    & 2(q_y q_z - q_w q_x) \\
    2(q_x q_y - q_w q_y)
    & 2(q_y q_z + q_w q_x)
    & q_w^2 - q_x^2 - q_y^2 + q_z^2
  \end{bmatrix}



Rotation Matrix to Quaternion
-----------------------------

.. math::
  \begin{align}
    q_w &= \dfrac{\sqrt{1 + m_{11} + m_{22} + m_{33}}}{2} \\
    q_x &= \dfrac{m_{32} - m_{23}}{4 q_w} \\
    q_y &= \dfrac{m_{13} - m_{31}}{4 q_w} \\
    q_z &= \dfrac{m_{21} - m_{02}}{4 q_w}
  \end{align}

Note, while the equations seems straight forward in practice, however,the trace
of the rotation matrix need to be checked inorder to guarantee correctness.


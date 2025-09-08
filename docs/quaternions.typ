#set page(columns: 2, margin: 0.5in)
#set text(
  font: "New Computer Modern",
  size: 9pt
)

= Quaternions

A quaternion, $q in RR^4$, generally has the following form

$
  q = q_w + q_x i + q_y j + q_z k,
$

where $q_w, q_x, q_y, q_z in RR$ and $i, j, k$ are the imaginary numbers
satisfying

$
    &i^2
    = j^2
    = k^2
    = i j k
    = -1 \
    i j = -j i &= k,
    j k = -k j = i,
    k i = -i k = j
$

corresponding to the Hamiltonian convention. The quaternion can be written as a
4 element vector consisting of a _scalar_ part, $q_w$, and _vector_ part $q_v
= (q_x, q_y, q_z)^T$ as,

$
  q = vec( q_w, q_x, q_y, q_z )
$

There are other quaternion conventions, for example, the JPL convention. A
more detailed discussion between Hamiltonian and JPL quaternion convention is
discussed in /* \cite{Sola2017} */


=== Sum

Let $p$ and $q$ be two quaternions, the sum of both
quaternions is,

$
  p plus.minus q =
  mat(p_w ; p_x ; p_y ; p_z)
  plus.minus
  mat(q_w ; q_x ; q_y ; q_z) =
  mat(p_w plus.minus q_w ; p_v plus.minus q_v).
$

The sum between two quaternions $p$ and $q$ is
_commutative_ and _associative_.

$
  p + q = q + p \
  p + (q + r) = (p + q) + r
$


=== Product

The quaternion multiplication (or product) of two quaternions $p$ and
$q$, denoted by $times.circle$ is defined as

$
  p times.circle q
    &=
    (p_w + p_x i + p_y j + p_z k)
    (q_w + q_x i + q_y j + q_z k) \
    &=
    mat(
      &(p_w q_w - p_x q_x - p_y q_y - p_z q_z)& ;
      &(p_w q_x + p_x q_w + p_y q_z - p_z q_y)& i;
      &(p_w q_y - p_y q_w + p_z q_x + p_x q_z)& j;
      &(p_w q_z + p_z q_w - p_x q_y + p_y q_x)& k;
    ) \
    &=
    mat(
      p_w q_w - p_x q_x - p_y q_y - p_z q_z ;
      p_w q_x + p_x q_w + p_y q_z - p_z q_y ;
      p_w q_y - p_y q_w + p_z q_x + p_x q_z ;
      p_w q_z + p_z q_w - p_x q_y + p_y q_x ;
    ) \
    &=
    mat(
      p_w q_w - p_v^T q_v ;
      p_w q_v + q_w p_v + p_v times q_v
    ).
$

The quaternion product is _not commutative_ in the general case. There
are exceptions to the general non-commutative rule, where either $p$ or
$q$ is real such that $p_v times q_v = 0$, or when both
$p_v$ and $q_v$ are parallel, $p_v || q_v$. Only in
these cirmcumstances is the quaternion product commutative.,

$
  {p times.circle q eq.not q times.circle p} .
$

The quaternion product is however _associative_,

$
  p times.circle (q times.circle r)
  = (p times.circle q) times.circle r
$

and distributive over the sum

$
  p times.circle (q + r) =
  p times.circle q + p times.circle r
  quad "and" quad
  (p times.circle q) + r =
  p times.circle r + q times.circle r
$

The quaternion product can alternatively be expressed in matrix form as

$
  p times.circle q = [p]_L thick q
  quad "and" quad
  p times.circle q = [q]_R thick p thin ,
$

where $[p]_L$ and $[q]_R$ are the left and right quaternion-product matrices
which are derived from
/*\eqref{eq:quaternion_product}*/,

$
  [p]_L =
  mat(
    p_w , -p_x , -p_y , -p_z ;
    p_x , p_w , -p_z , p_y ;
    p_y , p_z , p_w , -p_x ;
    p_z , -p_y , p_x , p_w
  ),
  quad
  [q]_R =
  mat(
    q_w , -q_x , -q_y , -q_z ;
    q_x , q_w , q_z , -q_y ;
    q_y , -q_z , q_w , q_x ;
    q_z , q_y , -q_x , q_w
  ),
$

or inspecting a compact form can be derived as,

$
  [p]_L =
  mat(
    0 , -p_v^T ;
    p_w I_"3 times 3" + p_v ;
    p_w I_"3 times 3" -or( p_v)
  )
$

and

$
  [q]_R =
  mat(
    0 , -q_v^T ;
    q_w I_"3 times 3" + q_v ;
    q_w I_"3 times 3" -or q_v
  ),
$

/*

where $\vee{\bullet}$ is the skew operator that produces a matrix cross
product matrix, and is defined as,

$
  \veev} =
  mat(
    0,     -v_{3},  v_{2} ;
    v_{3},      0, -v_{1} ;
    -v_{2}, v_{1},     0
  ),
  quad
  v} \in \real^{3}
$

*/


=== Conjugate

/*
The conjugate operator for quaternion, ${(\bullet)}^{\ast}$, is defined
as

$
  q^ast
  =
  mat(
    q_w ;
    - q_v
  )
  =
  mat(
    q_w ;
    - q_x ;
    - q_y ;
    - q_z
  ).
$

This has the properties

$
  q times.circle q^"-1"
  = q^"-1" times.circle q
  = q_{w}^{2} + q_{x}^{2} + q_{y}^{2} + q_{z}^{2}
  =
  mat(
    q_{w}^{2} + q_{x}^{2} + q_{y}^{2} + q_{z}^{2} ;
    0}
  ),
$

and

$
  (p times.circle q)^{\ast}
  = q^{\ast} times.circle p^{\ast}.
$



=== Norm

The norm of a quaternion is defined by

$
  \norm{q &= \sqrt{q times.circle q^{\ast}} ;
    &= \sqrt{q^{\ast} times.circle q ;
    &= \sqrt{q_{w}^{2} + q_{x}^{2} + q_{y}^{2} + q_{z}^{2}}
    thin \in \real,
$

and has the property

$
  \norm{p times.circle q} =
  \norm{q times.circle p} =
  \norm{p} \norm{q}
$


=== Quaternion from Two Vectors

TODO: Need to reword the beginning.
Using the properties of the cross and dot product

$
  \begin{align}
    u} \cdot v} &=
      \norm{u}} \normv} \cos \theta ;
    \norm{u} times v}} &=
      \norm{u}} \normv} \norm{\sin \theta} ,
  \end{align}
$

the axis angle, $\boldsymbol{\theta} \in \real^{3}$, can be obtained from
$u}$ and $v}$ with

$
  \begin{align}
    %-- Axis-angle
    \boldsymbol{\theta} &= \theta e} ;
    % -- Angle
    \theta &= \cos^"-1"(
      \dfrac{u} \cdot v}}
            {\norm{u}} \normv}}
    ) quad , thin \theta \in \real ;
    % -- Axis
    e} &=
      \dfrac{u} times v}}{\norm{u} times v}}}
      quad , thin e} \in \real^{3}
  \end{align}
$

where $e}$ is the unit vector that defines the rotation axis and
$\theta$ is the rotation angle about $e}$. Once the axis angle,
$\boldsymbol{\theta}$, is obtained a quaternion can be formed

$
  q =
    \cos \dfrac{\theta}{2}
    + i) \sin \dfrac{\theta}{2} e_{x}
    + j) \sin \dfrac{\theta}{2} e_{y}
    + k) \sin \dfrac{\theta}{2} e_{z}
$


=== Example: Attitude from gravity and accelerometer vectors

In robotics knowing the attitude of the system is often required. An
Inertial Measurement Unit (IMU) is commonly used to obtain this information.
Using the method described previously, a gravity vector along with an
accelerometer measurement vector can be used to obtain an attitude in form of a
quaternion.

Let $g} \in \Real{3}$ be the gravity vector, and $a}_{m} \in
\Real{3}$ be the accelerometer measurement from an IMU. With the two vectors
$g}$ and $a}_{m}$ a quaternion $q_{WS}$ expressing the rotation
of the IMU sensor frame, $\frame_{S}$, with respect to the world frame,
$\frame_{W}$, can be calculated given that values for $g}$ and
$a}_{m}$ are known. For example let

$
  \begin{align}
    % -- Gravity vector
    g} &= mat( 0 & 0 & -9.81 )^T ;
    % -- Accelerometer measurement vector
    a}_{m} &=
      mat(
        9.2681 &
        -0.310816 &
        -3.14984
        )^T
    ,
  \end{align}
$

taken from the first measurement of the <code>imu_april</code> calibration
sequence of the EuRoC MAV dataset.

Before calculating the axis-angle, however, it should be noted that when an
accelerometer is at rest the measurement reading in the z-axis is positive
instead of negative. The reason is accelerometers measures acceleration by
measuring the displacement of a proof mass that is suspended with springs. For
example, if gravity is ignored and the accelerometer moves upwards, the proof
mass will be displaced towards the bottom of the accelerometer. This is
interpreted as an acceleration in the upwards direction, and so when the
accelerometer is at rest on a flat surface, gravity pulls on the proof mass
yeilding a positive measurement in the upwards direction. To resolve this issue
the gravity vector is negated, and so $u} = -g}$ and
$v} = a}_{m}$. Using :eq:$axis_angle$ the axis-angle obtained
is:

$
  \begin{align}
    % -- Axis-Angle
    \theta &= 1.8982 ;
    e} &= \Transpose{
      mat(
        0.03352 &
        0.99944 &
        0.00000
      )
    }
  \end{align}
$

Finally the quaternion, $q_{WS}$, can be calculated using
:eq:$axis_angle_to_quaternion$ resulting in

$
  \begin{align}
    q_{WS} = \Transpose{
      mat(
        0.58240 &
        0.02725 &
        0.81245 &
        0.00000
      )
    } thin .
  \end{align}
$



=== Quaternion to Rotation Matrix

$
  \rot\{q \} = mat(
    q_w^2 + q_x^2 - q_y^2 - q_z^2
    & 2(q_x q_y - q_w q_z)
    & 2(q_x q_z + q_w q_y) ;
    2(q_x q_y + q_w q_z)
    & q_w^2 - q_x^2 + q_y^2 - q_z^2
    & 2(q_y q_z - q_w q_x) ;
    2(q_x q_y - q_w q_y)
    & 2(q_y q_z + q_w q_x)
    & q_w^2 - q_x^2 - q_y^2 + q_z^2
  )
$


=== Rotation Matrix to Quaternion

$
  \begin{align}
    q_w &= \dfrac{\sqrt{1 + m_{11} + m_{22} + m_{33}}}{2} ;
    q_x &= \dfrac{m_{32} - m_{23}}{4 q_w} ;
    q_y &= \dfrac{m_{13} - m_{31}}{4 q_w} ;
    q_z &= \dfrac{m_{21} - m_{02}}{4 q_w}
  \end{align}
$

Note, while the equations seems straight forward in practice, however,the trace
of the rotation matrix need to be checked inorder to guarantee correctness.

*/

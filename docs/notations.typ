#set page(columns: 2, margin: 0.5in)
#set text(
  font: "New Computer Modern",
  size: 9pt
)

= Notations

A large part of robotics is about developing machines that perceives and
interact with the environment. For that robots use sensors to collect and
process data, and knowing what the data describes is of important for the robot
to navigate and interact with the environment. The following notes will be
using the frame notation as described by Paul Furgale (see
https://tinyurl.com/y635m93l). The aim is to mitigate the ambiguity that arises
when describing robot poses, sensor data and more.

A vector expressed in the world frame, $cal(F)_"W"$, is written as $p_"W"$. Or
more precisely if the vector describes the position of the camera frame,
$cal(F)_"C"$, expressed in $cal(F)_"W"$, the vector can be written as $p_"WC"$.
The left hand subscripts indicates the coordinate system the vector is
expressed in, while the right-hand subscripts indicate the start and end
points. For brevity if the vector has the same start point as the frame to
which it is expressed in, the same vector can be written as $p_"WC"$. Similarly
a transformation of a point from $cal(F)_"W"$ to $cal(F)_"C"$ can be
represented by a homogeneous transform matrix, $T_"WC"$, where its rotation
matrix component is written as $C_"WC"$ and the translation component written
as $p_"WC"$. A rotation matrix that is parametrized by quaternion $q_"WC"$ is
written as $C{q_"WC"}$.

#let vel = $attach(thin v thin, b:"WB", bl: "W")$
$
& "Position" quad & attach(r, b:"WB", bl: W) \
  & "Velocity" quad & vel \
  & "Acceleration" quad & a_"WB" \
  & "Angular velocity" quad & w_"WB" \
  & "Rotation" quad & C_"WB" \
  & "Transform" quad & T_"WB" \
$

#figure(
  image("images/notations/frame_diagram_example.png", width: 80%),
  caption: [ Example of a frame diagram ],
)

#figure(
  image("images/notations/notation_translation.png", width: 80%),
)

#figure(
  image("images/notations/notation_rotation.png", width: 80%),
)

#figure(
  image("images/notations/notation_transform.png", width: 80%),
)

#figure(
  image("images/notations/notation_angular_velocity.png", width: 80%),
)

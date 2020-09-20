graphics_toolkit("fltk");

function T = tf(rot, trans)
  assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
  assert(size(trans) == [3, 1]);

  C = rot;
  if size(rot) == [4, 1]
    C = quat2rot(rot);
  endif

  T = eye(4, 4);
  T(1:3, 1:3) = C;
  T(1:3, 4) = trans;
endfunction

function R = quat2rot(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;
  qw2 = qw**2;

  # Homogeneous form
  R11 = qw2 + qx2 - qy2 - qz2;
  R12 = 2 * (qx * qy - qw * qz);
  R13 = 2 * (qx * qz + qw * qy);

  R21 = 2 * (qx * qy + qw * qz);
  R22 = qw2 - qx2 + qy2 - qz2;
  R23 = 2 * (qy * qz - qw * qx);

  R31 = 2 * (qx * qz - qw * qy);
  R32 = 2 * (qy * qz + qw * qx);
  R33 = qw2 - qx2 - qy2 + qz2;

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function hp = homogeneous(p)
  hp = [p; ones(1, columns(p))];
endfunction

function r = tf_trans(tf)
  r = tf(1:3, 4);
endfunction

function draw_frame(T_WS, scale=0.1)
  r_WS = tf_trans(T_WS);
  origin = r_WS;

  x_axis = T_WS * homogeneous(scale * [1; 0; 0]);
  y_axis = T_WS * homogeneous(scale * [0; 1; 0]);
  z_axis = T_WS * homogeneous(scale * [0; 0; 1]);

  % Draw x-axis
  plot3([origin(1), x_axis(1)], ...
        [origin(2), x_axis(2)], ...
        [origin(3), x_axis(3)], 'r',
        "linewidth", 5)

  % Draw y-axis
  plot3([origin(1), y_axis(1)], ...
        [origin(2), y_axis(2)], ...
        [origin(3), y_axis(3)], 'g',
        "linewidth", 5)

  % Draw z-axis
  plot3([origin(1), z_axis(1)], ...
        [origin(2), z_axis(2)], ...
        [origin(3), z_axis(3)], 'b',
        "linewidth", 5)
endfunction

q_WV = [0.99993; -0.005; -0.003; 0.009];
r_WV = [-0.03905; 0.2724; 0.2725];
T_WV = tf(q_WV, r_WV);

T_VB = [
  1.0, 0.0, 0.0, 0.0;
  0.0, 1.0, 0.0, 0.04;
  0.0, 0.0, 1.0, -0.07;
  0.0, 0.0, 0.0, 1.0;
]

T_WB = T_WV * T_VB;
r_WB = T_WB(1:3, 4)


figure(1);
hold on;
grid on;
view(3);
draw_frame(T_WV, 0.1);
draw_frame(T_WB, 0.1);
% draw_frame(T_WB * T_BV, 0.1);
% draw_frame(T_WB, 0.1);

xlabel("x");
ylabel("y");
zlabel("z");
axis('equal');
ginput();

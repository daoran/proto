mav = {};
mav.r_WB = [0.0; 0.0; 0.0];
mav.q_WB = [0.0; 0.0; 0.0];
mav.w_WB = [0.0; 0.0; 0.0];
mav.v_WB = [0.0; 0.0; 0.0];
mav.a_WB = [0.0; 0.0; 0.0];
mav.C_T = 1.0;
mav.C_D = 0.0;
mav.C_R = 0.0;
mav.C_M = 1.0;
mav.e_ZB = [0.0; 0.0; 1.0];

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

function mav = mav_update(mav, dt)
  F_G = [0.0; 0.0; -9.81];
  C_WB = quat2rot(mav.q_WB);

  F_T = w**2 * mav.C_T * mav.e_ZB;
  F_D = -w * mav.C_D * (mav.v_WB - (mav.v_WB * mav.e_ZB) * mav.e_ZB);
  M_R = w * mav.C_R * (mav.v_WB - (mav.v_WB * mav.e_ZB) * mav.e_ZB);
  M_D = -epsilon * mav.C_M * F_T;

  a_WB = (4 * (C_WB * (F_T + F_D)) + F_G) / m;
endfunction

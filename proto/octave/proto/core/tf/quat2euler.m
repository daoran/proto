function rpy = quat2euler(q)
  % Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  % Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  % Princeton University Press, 1999. Print.
  % Page 168, "Quaternion to Euler Angles"
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qw2 = qw**2;
  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;

  m11 = (2 * qw**2) + (2 * qx**2) - 1;
  m12 = 2 * (qx * qy + qw * qz);
  m13 = 2 * qx * qz - 2 * qw * qy;
  m23 = 2 * qy * qz + 2 * qw * qx;
  m33 = (2 * qw**2) + (2 * qz**2) - 1;

  psi = atan2(m12, m11);
  theta = asin(-m13);
  phi = atan2(m23, m33);

  rpy = [phi; theta; psi];
endfunction

function q = euler2quat(euler)
  phi = euler(1);
  theta = euler(2);
  psi = euler(3);

  c_phi = cos(phi / 2.0);
  c_theta = cos(theta / 2.0);
  c_psi = cos(psi / 2.0);
  s_phi = sin(phi / 2.0);
  s_theta = sin(theta / 2.0);
  s_psi = sin(psi / 2.0);

  % qw = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi;
  % qx = c_psi * c_theta * s_psi - s_psi * s_theta * c_phi;
  % qy = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi;
  % qz = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi;

  qx = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
  qy = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
  qz = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
  qw = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;

  mag = sqrt(qw**2 + qx**2 + qy**2 + qz**2);
  q = [qx / mag; qy / mag; qz / mag; qw / mag;];
endfunction

function q = euler2quat(rpy)
  % Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
  % Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
  % Princeton University Press, 1999. Print.
  % Page 166-167, "Euler Angles to Quaternion"
  psi = rpy(3);   % Yaw
  theta = rpy(2); % Pitch
  phi = rpy(1);   % Roll

  c_phi = cos(phi / 2.0);
  c_theta = cos(theta / 2.0);
  c_psi = cos(psi / 2.0);
  s_phi = sin(phi / 2.0);
  s_theta = sin(theta / 2.0);
  s_psi = sin(psi / 2.0);

  qw = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi;
  qx = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi;
  qy = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi;
  qz = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi;

  mag = sqrt(qw**2 + qx**2 + qy**2 + qz**2);
  q = [qw / mag; qx / mag; qy / mag; qz / mag;];
endfunction

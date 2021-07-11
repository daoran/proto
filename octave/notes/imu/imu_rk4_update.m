function x_imu = imu_rk4_update(x_imu, acc, gyr, dt)
  g = x_imu.g;
  ba = x_imu.ba;
  bg = x_imu.bg;
  na = zeros(3, 1);
  ng = zeros(3, 1);

  C_WS_k = x_imu.C_WS;
  q_WS_k = rot2quat(C_WS_k);
  v_WS_k = x_imu.v_WS;
  r_k = x_imu.p_WS;

  w = gyr - bg;
  a = acc - ba;

  % Runge-Kutta 4th Order
  % -- Integrate orientation at time k + dt (kpdt: k plus dt)
  q_WS_kpdt = quat_integrate(q_WS_k, w, dt);
  C_WS_kpdt = quat2rot(q_WS_kpdt);
  % -- Integrate orientation at time k + dt / 2 (kphdt: k plus half dt)
  q_WS_kphdt = quat_integrate(q_WS_k, w, dt / 2);
  C_WS_kphdt = quat2rot(q_WS_kphdt);
  % -- k1 = f(tn, yn)
  k1_v_dot = C_WS_k * a - g;
  k1_p_dot = v_WS_k;
  % -- k2 = f(tn + dt / 2, yn + k1 * dt / 2)
  k2_v_dot = C_WS_kphdt * acc - g;
  k2_p_dot = v_WS_k + k1_v_dot * dt / 2;
  % -- k3 = f(tn + dt / 2, yn + k2 * dt / 2)
  k3_v_dot = C_WS_kphdt * acc - g;
  k3_p_dot = v_WS_k + k2_v_dot * dt / 2;
  % -- k4 = f(tn + dt, tn + k3 * dt)
  k4_v_dot = C_WS_kpdt * acc - g;
  k4_p_dot = v_WS_k + k3_v_dot * dt;

  x_imu.C_WS = C_WS_kpdt;
  x_imu.v_WS += dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
  x_imu.p_WS += dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);
endfunction

function x_imu = imu_euler_update(x_imu, acc, gyr, dt)
  g = x_imu.g;
  ba = x_imu.ba;
  bg = x_imu.bg;

  C_WS_i = x_imu.C_WS;
  v_WS_i = x_imu.v_WS;

  w = gyr - bg;
  a = acc - ba;
  dt_sq = dt * dt;

  x_imu.p_WS += (v_WS_i * dt) + (0.5 * (C_WS_i * a - g) * dt_sq);
  x_imu.v_WS += ((C_WS_i * a) - g) * dt;
  x_imu.C_WS *= Exp(w * dt);
endfunction

function x_imu = imu_state_init()
  x_imu.p_WS = zeros(3, 1);
  x_imu.v_WS = zeros(3, 1);
  x_imu.C_WS = eye(3);
  x_imu.ba = zeros(3, 1);
  x_imu.bg = zeros(3, 1);
  x_imu.g = [0; 0; 9.81];
endfunction

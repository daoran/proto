function imu = imu_create()
  imu.q_IG = [1.0; 0.0; 0.0; 0.0];
  imu.b_g = [0.0; 0.0; 0.0];
  imu.v_IG = [0.0; 0.0; 0.0];
  imu.b_a = [0.0; 0.0; 0.0];
  imu.p_IG = [0.0; 0.0; 0.0];
endfunction

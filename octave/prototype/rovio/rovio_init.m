function rovio = rovio_init()
  rovio = {};

  % Gravity vector
  rovio.g = [0; 0; 9.81];

  % IMU
  rovio.imu.r = [0.0; 0.0; 0.0];
  rovio.imu.v = [0.0; 0.0; 0.0];
  rovio.imu.q = [1.0; 0.0; 0.0; 0.0];
  rovio.imu.b_f = [0.0; 0.0; 0.0];
  rovio.imu.b_w = [0.0; 0.0; 0.0];

  % IMU-Camera Extrinsics
  rovio.imu_cam.p = [0.0; 0.0; 0.0];
  rovio.imu_cam.q = [1.0; 0.0; 0.0; 0.0];

  % Features
  rovio.features = [];
endfunction

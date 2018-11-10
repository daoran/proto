function camera = camera_create(T_WC)
  camera.fov = 60;
  camera.T_WC = T_WC;
  camera.resolution = [680; 480];

  # Calculate theoretical intrinsics using image resolution and fov
  image_width = camera.resolution(1);
  image_height = camera.resolution(2);
  cx = image_width / 2.0;
  cy = image_height / 2.0;
  fx = (image_width / 2.0) / tan(deg2rad(camera.fov / 2.0));
  fy = (image_height / 2.0) / tan(deg2rad(camera.fov / 2.0));
  camera.K = [fx, 0.0, cx;
              0.0, fy, cy;
              0.0, 0.0, 1.0;];
endfunction

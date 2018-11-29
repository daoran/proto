function camera = camera_create(T_WC)
  camera.fov = 60;
  camera.T_WC = T_WC;
  camera.resolution = [680; 480];

  # Calculate theoretical intrinsics using image resolution and fov
  image_width = camera.resolution(1);
  image_height = camera.resolution(2);
  cx = image_width / 2.0;
  cy = image_height / 2.0;
  fx = focal_length(image_width, camera.fov);
  fy = focal_length(image_height, camera.fov);
  camera.K = pinhole_K([fx, fy, cx, cy]);
endfunction

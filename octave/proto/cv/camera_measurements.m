function [z, point_ids] = camera_measurements(camera, T_WC, points_W)
  assert(size(T_WC) == [4, 4]);
  assert(rows(points_W) == 3);
  assert(columns(points_W) > 0);

  # Form projection matrix
  T_CW = inv(T_WC);
  C_CW = T_CW(1:3, 1:3);
  r_CW = T_CW(1:3, 4);
  K = pinhole_K(camera.param(1:4));
  P = K * [C_CW, r_CW];

  # Setup
  image_width = camera.resolution(1);
  image_height = camera.resolution(2);
  nb_points = columns(points_W);

  # Check points
  z = [];
  point_ids = [];

  for i = 1:nb_points
    # Project point to image plane
    hp_W = homogeneous(points_W(:, i));
    x = P * hp_W;

    # Check to see if point is infront of camera
    if x(3) < 1e-4
      continue;
    endif

    # Normalize projected ray
    x(1) = x(1) / x(3);
    x(2) = x(2) / x(3);
    x(3) = x(3) / x(3);
    z_hat = [x(1); x(2)];

    # Check to see if ray is within image plane
    x_ok = (x(1) < image_width) && (x(1) > 0.0);
    y_ok = (x(2) < image_height) && (x(2) > 0.0);
    if x_ok && y_ok
      z = [z, z_hat];
      point_ids = [point_ids, i];
    endif
  endfor
  assert(columns(z) == length(point_ids));
endfunction

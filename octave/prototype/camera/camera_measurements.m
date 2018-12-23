function [z_out, points_out] = camera_measurements(K, resolution, T_WC, points_W)
  # Form projection matrix
  T_CW = inv(T_WC);
  R_CW = T_CW(1:3, 1:3);
  t_CW = T_CW(1:3, 4);
  P = K * [R_CW, t_CW];

  # Setup
  image_width = resolution(1);
  image_height = resolution(2);
  nb_points = columns(points_W);

  # Check points
  z_out = [];
  points_out = [];

  for i = 1:nb_points
    # Project point to image plane
    p_W = points_W(1:3, i);
    hp_W = [p_W; 1.0];
    x = P * hp_W;

    # Check to see if point is infront of camera
    if x(3) < 1.0
      continue;
    endif

    # Normalize projected ray
    x(1) = x(1) / x(3);
    x(2) = x(2) / x(3);
    x(3) = x(3) / x(3);
    z = [x(1); x(2)];

    # Check to see if ray is within image plane
    x_ok = (x(1) < image_width) && (x(1) > 0.0);
    y_ok = (x(2) < image_height) && (x(2) > 0.0);
    if x_ok && y_ok
      z_out = [z_out, z];
      points_out = [points_out, hp_W(1:3)];
    endif
  endfor
endfunction

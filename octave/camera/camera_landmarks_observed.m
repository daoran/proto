function [z_data, landmark_data] = camera_landmarks_observed(camera, landmarks)
  # Form projection matrix
  K = camera.K;
  T_CW = inv(camera.T_WC);
  R_CW = T_CW(1:3, 1:3);
  t_CW = T_CW(1:3, 4);
  P = K * [R_CW, t_CW];

  # Setup
  image_width = camera.resolution(1);
  image_height = camera.resolution(2);
  nb_landmarks = columns(landmarks);

  # Check landmarks
  z_data = [];
  landmark_data = [];

  for i = 1:nb_landmarks
    # Project landmark to image plane
    landmark = landmarks(1:3, i);
    landmark = [landmark; 1.0];
    x = P * landmark;

    # Check to see if landmark is infront of camera
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
      z_data = [z_data, z];
      landmark_data = [landmark_data, landmark(1:3)];
    endif
  endfor
endfunction

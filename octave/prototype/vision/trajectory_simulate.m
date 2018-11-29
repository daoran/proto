function data = trajectory_simulate(camera, chessboard)
  # Trajectory - bezier position settings
  pos_init = [0.0; 0.0; 0.0];
  wp_0 = [1.0; 1.0; 0.0];
  wp_1 = [2.0; -1.0; 0.0];
  pos_end = [3.0; 0.0; 0.0];

  # Trajectory - bezier attitude settings
  rpy_init = [deg2rad(-90.0); 0.0; deg2rad(-90.0)];
  att_0 = [deg2rad(-90.0); 0.0; deg2rad(-90.0 - 5.0)];
  att_1 = [deg2rad(-90.0); 0.0; deg2rad(-90.0 + 5.0)];
  rpy_end = [deg2rad(-90.0); 0.0; deg2rad(-90.0)];

  # Initialize camera position and rotation
  T_WC = transform(euler321(rpy_init), pos_init);
  camera.T_WC = T_WC;

  # Generate trajectory
  landmarks = chessboard.corners;
  time_data = [0.0];
  pos_data = [pos_init];
  rpy_data = [rpy_init];
  [z_init, landmarks_init] = camera_landmarks_observed(camera, landmarks);
  z_data = {z_init};
  landmark_data = {landmarks_init};

  for t = 0.1:0.1:1.0
    # Update camera pose
    pos_t = bezier_cubic(pos_init, wp_0, wp_1, pos_end, t);
    rpy_t = bezier_cubic(rpy_init, att_0, att_1, rpy_end, t);
    camera.T_WC = transform(euler321(rpy_t), pos_t);

    # Check landmarks
    [z_t, landmarks_t] = camera_landmarks_observed(camera, landmarks);

    # Update data
    time_data = [time_data, t];
    pos_data = [pos_data, pos_t];
    rpy_data = [rpy_data, rpy_t];
    z_data(columns(z_data) + 1) = z_t;
    landmark_data(columns(landmark_data) + 1) = landmarks_t;
  endfor

  # Form simulation data
  data.time = time_data;
  data.camera = camera;
  data.chessboard = chessboard;
  data.camera_position = pos_data;
  data.camera_orientation = rpy_data;
  data.z_data = z_data;
  data.landmark_data = landmark_data;
endfunction

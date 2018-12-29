function data = trajectory_simulate(camera, chessboard)
  % Trajectory - bezier position settings
  pos_init = [0.0; 0.0; 0.0];
  wp_0 = [1.0; 1.0; 0.0];
  wp_1 = [2.0; -1.0; 0.0];
  pos_end = [3.0; 0.0; 0.0];

  % Trajectory - bezier attitude settings
  rpy_init = deg2rad([-90.0; 0.0; -90.0]);
  att_0 = deg2rad([-90.0; 0.0; -90.0 - 5.0]);
  att_1 = deg2rad([-90.0; 0.0; -90.0 + 5.0]);
  rpy_end = deg2rad([-90.0; 0.0; -90.0]);

  % Initialize camera position and rotation
  T_WC = tf(euler321(rpy_init), pos_init);
  T_WF = tf(euler321(deg2rad([0.0; -90.0; 0.0])), [5.0; 0.0; 0.0]);

  % Project fiducial points to world frame
  hp_F = homogeneous(chessboard.object_points);
  p_W = dehomogeneous(T_WF * hp_F);
  K = camera.K;
  res = camera.resolution;

  % Generate trajectory
  time_data = [];
  T_WC = {};
  z_data = {};
  p_data = {};
  time_index = 1;

  for t = 0.0:0.1:1.0
    % Update camera pose
    pos_t = bezier_cubic(pos_init, wp_0, wp_1, pos_end, t);
    rpy_t = bezier_cubic(rpy_init, att_0, att_1, rpy_end, t);
    cam_pose_t = tf(euler321(rpy_t), pos_t);

    % Check landmarks
    [z_out, points_out] = camera_measurements(K, res, cam_pose_t, p_W);

    % Record data
    time_data = [time_data, t];
    T_WC{time_index} = cam_pose_t;
    z_data(columns(z_data) + 1) = z_out;
    p_data(columns(p_data) + 1) = points_out;
    time_index++;
  endfor

  % Form simulation data struct
  data.time = time_data;
  data.camera = camera;
  data.chessboard = chessboard;
  data.T_WF = T_WF;
  data.T_WC = T_WC;
  data.z_data = z_data;
  data.p_data = p_data;
endfunction

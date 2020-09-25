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

  % Project fiducial points to world frame
  C_WT = euler321(deg2rad([0.0; -90.0; 0.0]));
  q_WT = euler2quat(deg2rad([0.0; -90.0; 0.0]));
  r_WT = [5.0; 0.0; 0.0];
  T_WT = tf(C_WT, r_WT);
  hp_F = homogeneous(chessboard.object_points);
  p_W = dehomogeneous(T_WT * hp_F);

  % Generate trajectory
  time_data = [];
  q_WC = {};
  r_WC = {};
  z_data = {};
  p_data = {};
  time_index = 1;

  for t = 0.0:0.1:1.0
    % Update camera pose
    rpy_WC_k = bezier_cubic(rpy_init, att_0, att_1, rpy_end, t);
    q_WC_k = euler2quat(rpy_WC_k);
    C_WC_k = euler321(rpy_WC_k);
    r_WC_k = bezier_cubic(pos_init, wp_0, wp_1, pos_end, t);
    T_WC_k = tf(C_WC_k, r_WC_k);

    % Check landmarks
    [z_out, points_out] = camera_measurements(camera, T_WC_k, p_W);

    % Record data
    time_data = [time_data, t];
    q_WC{time_index} = q_WC_k;
    r_WC{time_index} = r_WC_k;
    z_data{time_index} = z_out;
    p_data{time_index} = points_out;
    time_index++;
  endfor

  % Form simulation data struct
  % -- Time, camera and chessboard data
  data.time = time_data;
  data.camera = camera;
  data.target = chessboard;
  % -- Fiducial pose
  data.q_WT = q_WT;
  data.r_WT = r_WT;
  % -- Camera poses
  data.q_WC = q_WC;
  data.r_WC = r_WC;
  % -- Image measurements and corresponding landmark points
  data.z_data = z_data;
  data.p_data = p_data;
endfunction

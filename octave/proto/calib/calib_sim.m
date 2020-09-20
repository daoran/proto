function data = calib_sim(calib_target, T_WT, camera, nb_poses, debug=false)
  % Generate camera poses
  [poses, calib_center] = calib_generate_random_poses(calib_target, nb_poses);
  nb_poses = length(poses);

  % Transform calibration grid points from target to world frame
  p_data = [];
  hp_T = homogeneous(calib_target.object_points);
  hp_W = T_WT * hp_T;
  p_data = dehomogeneous(hp_W);
  assert(rows(calib_target.object_points) == 3);
  assert(rows(p_data) == 3);

  % Decompose target pose to quaternion and translation vector
  C_WT = tf_rot(T_WT);
  r_WT = tf_trans(T_WT);
  q_WT = rot2quat(C_WT);
  assert(norm(quat2rot(q_WT) - C_WT) < 1e-4);

  % Simulation calibration data
  point_ids_data = {};
  z_data = {};
  q_WC = {};
  r_WC = {};
  cam_poses = {};

  for i = 1:nb_poses
    % Transform camera pose in target frame to world frame
    T_TC = poses{i};
    T_WC = T_WT * T_TC;
    cam_poses{i} = T_WC;

    % Decompose camera pose to quaternion and translation vector
    C_WC = tf_rot(T_WC);
    q_WC{i} = rot2quat(C_WC);
    r_WC{i} = tf_trans(T_WC);
    assert(norm(quat2rot(q_WC{i}) - C_WC) < 1e-4);

    % Project world points to camera
    K = camera.K;
    image_size = camera.resolution;
    [z, point_ids] = camera_measurements(K, image_size, T_WC, p_data);

    z_data{i} = z;
    point_ids_data{i} = point_ids;
  endfor

  % Form simulation data struct
  % -- Time, camera and calib_target data
  data.time = 1:nb_poses;
  data.camera = camera;
  data.target = calib_target;
  % -- Calibration target
  data.q_WT = q_WT;
  data.r_WT = r_WT;
  % -- Camera poses
  data.q_WC = q_WC;
  data.r_WC = r_WC;
  % -- Image measurements and corresponding landmark points
  data.z_data = z_data;
  data.point_ids_data = point_ids_data;
  data.p_data = p_data;

  % Visualize
  if debug
    figure()
    hold on;
    for i = 1:nb_poses
      calib_target_draw(calib_target, T_WT);
      T_WC = cam_poses{i};
      draw_camera(T_WC);
      draw_frame(T_WC, 0.05);
    endfor
    view(3);
    axis("equal");
    xlabel("x [m]");
    ylabel("y [m]");
    zlabel("z [m]");
    ginput();
  endif
endfunction

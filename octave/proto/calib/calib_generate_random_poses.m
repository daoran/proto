function [poses, calib_center] = calib_generate_random_poses(calib_target,
                                                             nb_poses)
  calib_width = (calib_target.nb_rows - 1.0) * calib_target.tag_size;
  calib_height = (calib_target.nb_cols - 1.0) * calib_target.tag_size;
  calib_center = [calib_width / 2.0; calib_height / 2.0; 0.0];

  % Settings
  angle_range = [-20.0, 20.0];
  x_range = [-0.5, 0.5];
  y_range = [-0.5, 0.5];
  z_range = [0.5, 0.7];

  % For each position create a camera pose that "looks at" the AprilGrid
  % center in the target frame, T_TC.
  poses = {};
  pose_idx = 1;
  for i = 1:nb_poses
    % Generate random pose
    x = unifrnd(x_range(1), x_range(2));
    y = unifrnd(y_range(1), y_range(2));
    z = unifrnd(z_range(1), z_range(2));
    r_TC = calib_center + [x; y; z] ;
    T_TC = lookat(r_TC, calib_center);

    % Perturb the pose a little so it doesn't look at the center directly
    p = randf(angle_range);
    q = randf(angle_range);
    r = randf(angle_range);
    C_perturb = euler321(deg2rad([p, q, r]));
    r_perturb = zeros(3, 1);
    T_perturb = tf(C_perturb, r_perturb);

    poses{pose_idx} = T_perturb * T_TC;
    pose_idx++;
  endfor
endfunction

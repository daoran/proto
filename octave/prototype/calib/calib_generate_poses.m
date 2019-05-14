function [poses, calib_center] = calib_generate_poses(calib_target)
  calib_width = (calib_target.nb_rows - 1.0) * calib_target.tag_size;
  calib_height = (calib_target.nb_cols - 1.0) * calib_target.tag_size;
  calib_center = [calib_width / 2.0; calib_height / 2.0; 0.0];

  % Pose settings
  x_range = linspace(-0.3, 0.3, 5);
  y_range = linspace(-0.3, 0.3, 5);
  z_range = linspace(0.2, 0.5, 5);

  % Generate camera positions infrom of the AprilGrid target in the target frame,
  % r_TC.
  cam_positions = zeros(3, length(x_range) * length(y_range) * length(z_range));
  pos_idx = 1;
  for i = 1:length(x_range)
    for j = 1:length(y_range)
      for k = 1:length(z_range)
        x = x_range(i);
        y = y_range(j);
        z = z_range(k);
        r_TC = [x; y; z] + calib_center;  % Calib center as offset
        cam_positions(:, pos_idx) = r_TC;
        pos_idx += 1;
      endfor
    endfor
  endfor

  % For each position create a camera pose that "looks at" the AprilGrid
  % center in the target frame, T_TC.
  poses = {};
  pose_idx = 1;
  for i = 1:length(cam_positions)
    T_TC = lookat(cam_positions(:, i), calib_center);
    poses{pose_idx} = T_TC;
    pose_idx++;
  endfor
endfunction

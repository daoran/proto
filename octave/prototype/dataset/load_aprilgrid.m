function [retval, aprilgrid] = load_aprilgrid(data_path)
  csv_data = dlmread(data_path, ",", 1, 0);
  if rows(csv_data) == 0
    retval = -1;
    aprilgrid = {};
    return;
  end

  # Target properties
  aprilgrid.configured = csv_data(1, 1);
  aprilgrid.tag_rows = csv_data(1, 2);
  aprilgrid.tag_cols = csv_data(1, 3);
  aprilgrid.tag_size = csv_data(1, 4);
  aprilgrid.tag_spacing = csv_data(1, 5);

  # Observed keypoints
  aprilgrid.configured = csv_data(1, 1);
  aprilgrid.ts = csv_data(1, 6);
  aprilgrid.id = csv_data(1:4:end, 7);
  kp_x = csv_data(1:end, 8);
  kp_y = csv_data(1:end, 9);
  aprilgrid.keypoints = transpose([kp_x, kp_y]);

  # Estimated fiducuial pose
  aprilgrid.estimated = csv_data(1, 10);
  # -- AprilTag corners
  p_x = csv_data(1:end, 11);
  p_y = csv_data(1:end, 12);
  p_z = csv_data(1:end, 13);
  aprilgrid.points_CF = transpose([p_x, p_y, p_z]);
  # -- Quaternion q_WF
  q_w = csv_data(1, 14);
  q_x = csv_data(1, 15);
  q_y = csv_data(1, 16);
  q_z = csv_data(1, 17);
  aprilgrid.q_CF = [q_w; q_x; q_y; q_z];
  # -- Translation t_WF
  t_x = csv_data(1, 18);
  t_y = csv_data(1, 19);
  t_z = csv_data(1, 20);
  aprilgrid.r_CF = [t_x; t_y; t_z];
  # -- Form transform T_WF
  aprilgrid.T_CF = tf(quat2rot(aprilgrid.q_CF), aprilgrid.r_CF);

  retval = 0;
endfunction

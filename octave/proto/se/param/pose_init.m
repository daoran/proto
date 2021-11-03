function pose = pose_init(ts, data)
  assert(all(size(data) == [7, 1]) || all(size(data) == [4, 4]));

  % Convert pose data from 4x4 homogenous transformation matrix to
  % pose vector (rx, ry, rz, qx, qy, qz, qw)
  if all(size(data) == [4, 4])
    data = tf_param(data);
  endif

  pose.fixed = false;
  pose.type = "pose";
  pose.ts = ts;
  pose.param = data;
  pose.min_dims = 6;
endfunction

function extrinsics = extrinsics_init(ts, data)
  assert(size(data) == [7, 1] || size(data) == [4, 4]);

  % Convert pose data from 4x4 homogenous transformation matrix to
  % extrinsics vector (rx, ry, rz, qx, qy, qz, qw)
  if all(size(data) == [4, 4])
    data = tf_param(data);
  endif

  extrinsics.fixed = false;
  extrinsics.type = "extrinsics";
  extrinsics.ts = ts;
  extrinsics.param = data;
  extrinsics.min_dims = 6;
endfunction

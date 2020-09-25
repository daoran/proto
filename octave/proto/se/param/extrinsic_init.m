function extrinsic = extrinsic_init(ts, data)
  assert(size(data) == [7, 1] || size(data) == [4, 4]);

  % Convert pose data from 4x4 homogenous transformation matrix to
  % extrinsic vector (qw, qx, qy, qz, rx, ry, rz)
  if all(size(data) == [4, 4])
    data = [tf_quat(data); tf_trans(data)];
  endif

  extrinsic.type = "extrinsic";
  extrinsic.ts = ts;
  extrinsic.param = data;
  extrinsic.min_dims = 6;
endfunction

function param = tf_param(T)
  assert(all(size(T) == [4, 4]));

  trans = tf_trans(T);
  rx = trans(1);
  ry = trans(2);
  rz = trans(3);

  rot = tf_quat(T);
  qw = rot(1);
  qx = rot(2);
  qy = rot(3);
  qz = rot(4);

  param = [rx; ry; rz; qx; qy; qz; qw];
endfunction

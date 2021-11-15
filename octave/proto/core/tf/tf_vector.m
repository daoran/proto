function v = tf_vector(T)
  r = tf_trans(T);
  q = tf_quat(T);

  qx = q(2);
  qy = q(3);
  qz = q(4);
  qw = q(1);

  v = [r(1); r(2); r(3); qx; qy; qz; qw];
endfunction

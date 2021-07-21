function param = tf_param(T)
  r = tf_trans(T);
  q = tf_quat(T);
  param = [q; r];
endfunction

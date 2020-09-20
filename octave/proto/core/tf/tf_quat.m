function q = tf_quat(tf)
  q = rot2quat(tf(1:3, 1:3));
endfunction

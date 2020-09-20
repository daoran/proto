function [C, r] = tf_decompose(tf)
  C = tf_rot(tf);
  r = tf_trans(tf);
endfunction

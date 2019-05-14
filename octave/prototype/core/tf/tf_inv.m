function retval = tf_inv(tf)
  C = tf(1:3, 1:3);
  r = tf(1:3, 4);
  retval = [C', -C' * r;
            0.0, 0.0, 0.0, 1.0];
endfunction

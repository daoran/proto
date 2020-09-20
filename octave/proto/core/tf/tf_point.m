function retval = tf_point(T, point)
  assert(all(size(point) == [3, 1]));
  hpoint = [point; 1.0];
  retval = (T * hpoint)(1:3);
endfunction

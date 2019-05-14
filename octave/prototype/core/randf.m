function retval = randf(bounds, sz=1)
  val_min = bounds(1);
  val_max = bounds(2);
  retval = (val_max - val_min) * rand(sz, 1) + val_min;
endfunction

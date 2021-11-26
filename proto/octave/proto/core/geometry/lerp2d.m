function p = lerp2d(p_start, p_end, t)
  assert(size(p0) == [3, 1]);
  assert(size(p1) == [3, 1]);
  assert(t <= 1.0 && t >= 0.0);

  x = lerp(p_start(1), p_end(1), t);
  y = lerp(p_start(2), p_end(2), t);

  p = [x; y];
endfunction
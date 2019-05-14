function dq = quat_delta(dalpha)
  half_norm = 0.5 * norm(dalpha);
  vector = sinc(half_norm) * 0.5 * dalpha;
  scalar = cos(half_norm);
  dq = [scalar; vector];
endfunction

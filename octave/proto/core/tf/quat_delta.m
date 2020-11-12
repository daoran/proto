function dq = quat_delta(dalpha)
  half_norm = 0.5 * norm(dalpha);
  scalar = cos(half_norm);
  vector = sinc(half_norm) * 0.5 * dalpha;
  dq = [scalar; vector];
endfunction

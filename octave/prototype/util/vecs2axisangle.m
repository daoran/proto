function axis_angle = vecs2axisangle(u, v)
  angle = acos(transpose(u) * v);
  ax = normalize(cross(u, v));
  axis_angle = ax * angle;
endfunction

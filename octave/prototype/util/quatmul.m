function result = quatmul(p, q)
  p_w = p(1);
  p_x = p(2);
  p_y = p(3);
  p_z = p(4);

  q_w = q(1);
  q_x = q(2);
  q_y = q(3);
  q_z = q(4);

  w = p_w * q_w - p_x * q_x - p_y * q_y - p_z * q_z;
  x = p_w * q_x + q_x * p_w + p_y * q_z - p_z * q_y;
  y = p_w * q_y - p_y * q_w + p_z * q_x + p_x * q_z;
  z = p_w * q_z + p_z * q_w - p_x * q_y + p_y * q_x;

	result = [w; x; y; z];
endfunction

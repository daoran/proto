function [theta, phi] = point2bearing(h_W)
  x = h_W(1);
  y = h_W(2);
  z = h_W(3);

  theta = atan2(x, z);
  phi = atan2(-y, sqrt(x * x + z * z));
endfunction

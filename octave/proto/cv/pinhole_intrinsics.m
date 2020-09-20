function [fx, fy, cx, cy] = pinhole_intrinsics(K)
  fx = K(1, 1);
  fy = K(2, 2);
  cx = K(1, 3);
  cy = K(2, 3);
endfunction

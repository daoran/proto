function K = pinhole_K(intrinsics)
  fx = intrinsics(1);
  fy = intrinsics(2);
  cx = intrinsics(3);
  cy = intrinsics(4);

  K = eye(3);
  K(1, 1) = fx;
  K(2, 2) = fy;
  K(1, 3) = cx;
  K(2, 3) = cy;
endfunction

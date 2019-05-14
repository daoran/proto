function R = rvec2rot(rvec)
  % If small rotation
  theta = sqrt(rvec(:)'*rvec(:));  % = norm(rvec), but faster
  if theta < eps
    R = [1, -rvec(3), rvec(2);
          rvec(3), 1, -rvec(1);
          -rvec(2), rvec(1), 1];
    return
  end

  % Convert rvec to rotation matrix
  rvec = rvec / theta;
  x = rvec(1);
  y = rvec(2);
  z = rvec(3);

  c = cos(theta);
  s = sin(theta);
  C = 1 - c;

  xs = x * s;
  ys = y * s;
  zs = z * s;

  xC = x * C;
  yC = y * C;
  zC = z * C;

  xyC = x * yC;
  yzC = y * zC;
  zxC = z * xC;

  R = [x * xC + c, xyC - zs, zxC + ys;
       xyC + zs, y * yC + c, yzC - xs;
       zxC - ys, yzC + xs, z * zC + c];
  return
endfunction

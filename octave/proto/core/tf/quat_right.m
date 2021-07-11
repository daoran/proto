function R = quat_right(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  R = [qw, -qx, -qy, -qz;
       qx, qw, qz, -qy;
       qy, -qz, qw, qx;
       qz, qy, -qx, qw];
endfunction

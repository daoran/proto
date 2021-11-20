function L = quat_left(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  L = [qw, -qx, -qy, -qz;
       qx, qw, -qz, qy;
       qy, qz, qw, -qx;
       qz, -qy, qx, qw];
endfunction

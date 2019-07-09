function r = quatrmul(p, q)
  assert(size(p) == [4, 1]);
  assert(size(q) == [4, 1]);

  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  rprod = [
    qw, -qx, -qy, -qz;
    qx, qw, qz, -qy;
    qy, -qz, qw, qx;
    qz, qy, -qx, qw;
  ];

  r = rprod * p;
endfunction

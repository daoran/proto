function K = pinhole_K(params)
  fx = params(1);
  fy = params(2);
  cx = params(3);
  cy = params(4);

  K = [fx, 0, cx,
       0, fy, cy,
       0, 0, 1];
endfunction

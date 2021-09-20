function p = dlt(z, z_dash, P, P_dash)
  % Image point from the first and second frame
  % Here we assume the image points are un-distorted (ideal)
  x = z(1);
  y = z(2);
  x_dash = z_dash(1);
  y_dash = z_dash(2);

  % First three rows of P and P_dash
  P1T = P(1, :);
  P2T = P(2, :);
  P3T = P(3, :);
  P1T_dash = P_dash(1, :);
  P2T_dash = P_dash(2, :);
  P3T_dash = P_dash(3, :);

  % Form the A matrix of AX = 0 where X is the 3D point we are estimating
  % A = [x * P3T - P1T;
  %      y * P3T - P2T;
  %      x_dash * P3T_dash - P1T_dash;
  %      y_dash * P3T_dash - P2T_dash];

  A = [y * P3T - P2T;
       x * P3T - P1T;
       y_dash * P3T_dash - P2T_dash;
       x_dash * P3T_dash - P1T_dash];

  % Use SVD to solve AX = 0
  [_, _, V] = svd(A' * A);
  hp = V(:, 4);     % Get the best result from SVD (last column of V)

  hp = hp / hp(4);  % Normalize the homogeneous 3D point
  p = hp(1:3);      % Return only the first three components (x, y, z)
endfunction

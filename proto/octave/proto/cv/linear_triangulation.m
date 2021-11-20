function linear_triangulation(P_i, P_j, z_i, z_j)
  % Linear triangulation
  % This function is used to triangulate a single 3D point observed by two
  % camera frames (be it in time with the same camera, or two different cameras
  % with known extrinsics)

  % First three rows of P_i and P_j
  P1T = P_i(1, :);
  P2T = P_i(2, :);
  P3T = P_i(3, :);
  P1T_dash = P_j(1, :);
  P2T_dash = P_j(2, :);
  P3T_dash = P_j(3, :);

  % Image point from the first and second frame
  x = z_i(1);
  y = z_j(2);
  x_dash = z_j(1);
  y_dash = z_j(2);

  % Form the A matrix of AX = 0
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

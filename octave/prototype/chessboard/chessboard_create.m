function chessboard = chessboard_create(T_WT)
  # Checkerboard tag size
  tag_size = 0.2;

  # Create chessboard grid
  corners = zeros(3, 16);
  idx = 1;
  for i = 1:4
    for j = 1:4
      corners(1:2, idx) = [j - 1; i - 1];
      idx += 1;
    endfor
  endfor

  # Scale the corner positions according to tag_size
  corners = tag_size * corners;

  # Transform corners to world frame
  corners = T_WT * [corners; ones(1, 16)];
  corners = corners(1:3, :);

  chessboard.nb_corners = 16;
  chessboard.tag_size = tag_size;
  chessboard.corners = corners;
endfunction

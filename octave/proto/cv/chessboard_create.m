function chessboard = chessboard_create(nb_rows=6, nb_cols=7, tag_size=0.2)
  # Create chessboard grid
  nb_corners = nb_rows * nb_cols;
  object_points = zeros(3, nb_corners);
  idx = 1;
  for i = 1:nb_rows
    for j = 1:nb_cols
      object_points(1:2, idx) = [j - 1; i - 1];
      idx += 1;
    endfor
  endfor

  # Chessboard struct
  chessboard.nb_rows = nb_rows;
  chessboard.nb_cols = nb_cols;
  chessboard.nb_corners = nb_corners;
  chessboard.tag_size = tag_size;
  chessboard.object_points = tag_size * object_points;
endfunction

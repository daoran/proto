function calib_target = calib_target_init(nb_rows=6, nb_cols=7, tag_size=0.2)
  # Create calib_target grid
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
  calib_target.nb_rows = nb_rows;
  calib_target.nb_cols = nb_cols;
  calib_target.nb_corners = nb_corners;
  calib_target.tag_size = tag_size;
  calib_target.width = calib_target.nb_cols * calib_target.tag_size;
  calib_target.height = calib_target.nb_rows * calib_target.tag_size;
  calib_target.center = [((nb_cols - 1.0) / 2.0) * tag_size,
                         ((nb_rows - 1.0) / 2.0) * tag_size];
  calib_target.object_points = tag_size * object_points;
endfunction

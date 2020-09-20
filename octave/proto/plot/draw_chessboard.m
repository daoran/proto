function draw_chessboard(T_WF, chessboard, color="r")
  % Draw points
  hp_F = homogeneous(chessboard.object_points);
  hp_W = T_WF * hp_F;
  p_W = dehomogeneous(hp_W);
  scatter3(p_W(1, 1:end), p_W(2, 1:end), p_W(3, 1:end), color);

  % Draw squares
  tag_id = 0;
  tag_rows = chessboard.nb_rows - 1;
  tag_cols = chessboard.nb_cols - 1;
  nb_tags = tag_rows * tag_cols;

  for tag_id = 1:nb_tags
    i = floor(tag_id / tag_cols);
    if mod(tag_id, tag_cols) == 0
      i--;
    end

    % -- Setup
    btm_left = tag_id + i;
    btm_right = btm_left + 1;
    top_right = btm_left + chessboard.nb_cols + 1;
    top_left = btm_left + chessboard.nb_cols;

    % -- Bottom left -> bottom right
    plot3([p_W(1, btm_left), p_W(1, btm_right)],
          [p_W(2, btm_left), p_W(2, btm_right)],
          [p_W(3, btm_left), p_W(3, btm_right)],
          color);
    % -- Bottom right -> top right
    plot3([p_W(1, btm_right), p_W(1, top_right)],
          [p_W(2, btm_right), p_W(2, top_right)],
          [p_W(3, btm_right), p_W(3, top_right)],
          color);
    % -- Top right -> top left
    plot3([p_W(1, top_right), p_W(1, top_left)],
          [p_W(2, top_right), p_W(2, top_left)],
          [p_W(3, top_right), p_W(3, top_left)],
          color);
    % -- Top left -> bottom left
    plot3([p_W(1, top_left), p_W(1, btm_left)],
          [p_W(2, top_left), p_W(2, btm_left)],
          [p_W(3, top_left), p_W(3, btm_left)],
          color);
  endfor
endfunction

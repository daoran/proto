function trajectory_plot(data)
  % Draw camera
  for i = 1:length(data.time)
    draw_camera(data.T_WC{i});
    draw_frame(data.T_WC{i}, 0.2);
  endfor

  % Draw chessboard
  hp_F = homogeneous(data.chessboard.object_points);
  hp_W = data.T_WF * hp_F;
  p_W = dehomogeneous(hp_W);
  draw_points(p_W);
  draw_frame(data.T_WF, 0.2);
endfunction

function trajectory_plot(data)
  % Draw camera
  for i = 1:length(data.time)
    draw_camera(data.T_WC{i});
    draw_frame(data.T_WC{i}, 0.2);
  endfor

  % Draw chessboard
  draw_chessboard(data.T_WF, data.chessboard);
  draw_frame(data.T_WF, 0.2);
endfunction

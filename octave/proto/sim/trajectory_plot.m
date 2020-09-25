function trajectory_plot(data)
  % Draw camera
  for i = 1:length(data.time)
    C_WC = quat2rot(data.q_WC{i});
    r_WC = data.r_WC{i};
    T_WC = tf(C_WC, r_WC);
    draw_camera(T_WC);
    draw_frame(T_WC, 0.05);
  endfor

  % Draw chessboard
  C_WT = quat2rot(data.q_WT);
  r_WT = data.r_WT;
  T_WT = tf(C_WT, r_WT);
  calib_target_draw(data.target, T_WT);
endfunction

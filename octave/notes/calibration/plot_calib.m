function plot_calib(T_WS, T_SC0, p_W)
  figure(1);
  hold on;
  grid on;
  view(3);
  scatter3(p_W(1), p_W(2), p_W(3), 'filled')
  draw_frame(T_WS, 0.05);
  draw_frame(T_WS * T_SC0, 0.05);
  draw_camera(T_WS * T_SC0, scale=0.05, 'r');
  xlabel("x");
  ylabel("y");
  zlabel("z");
  axis('equal');
  ginput();
endfunction

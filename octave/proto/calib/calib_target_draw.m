function calib_target_draw(calib_target, T_WT)
  hp_T = homogeneous(calib_target.object_points);
  hp_W = T_WT * hp_T;

  scatter3(hp_W(1, :), hp_W(2, :), hp_W(3, :), "r");
  draw_frame(T_WT, calib_target.tag_size);
endfunction

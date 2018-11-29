function trajectory_plot(data)
  # Draw camera
  for i = 1:columns(data.time)
    cam_pos = data.camera_position(1:3, i);
    cam_rpy = data.camera_orientation(1:3, i);
    T_WC = transform(euler321(cam_rpy), cam_pos);
    data.camera.T_WC = T_WC;
    camera_draw(data.camera);
  endfor

  # Draw chessboard
  chessboard_draw(data.chessboard);
endfunction

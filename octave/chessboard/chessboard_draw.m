function chessboard_draw(chessboard)
  scatter3(chessboard.corners(1, :),
           chessboard.corners(2, :),
           chessboard.corners(3, :));
endfunction

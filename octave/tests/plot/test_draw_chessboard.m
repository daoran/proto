addpath(genpath("proto"));

chessboard = chessboard_create(5, 4);
T_WF = tf(euler321(deg2rad([0.0; -90.0; 0.0])), [5.0; 0.0; 0.0]);

debug = false;
if debug
  figure(1);
  hold on;
  draw_chessboard(T_WF, chessboard, color="r")
  axis "equal";
  view(3);
  ginput();
end

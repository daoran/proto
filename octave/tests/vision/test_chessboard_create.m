addpath(genpath("prototype"));

chessboard = chessboard_create();

assert(chessboard.nb_rows == 4);
assert(chessboard.nb_cols == 4);
assert(chessboard.nb_corners == 16);
assert(chessboard.tag_size == 0.2);
assert(rows(chessboard.object_points) == 3);
assert(columns(chessboard.object_points) == 16);

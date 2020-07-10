addpath(genpath("proto"));

chessboard = chessboard_create();

assert(chessboard.nb_rows != 0);
assert(chessboard.nb_cols != 0);
assert(chessboard.nb_corners > 0);
assert(chessboard.tag_size == 0.2);
assert(rows(chessboard.object_points) == 3);
assert(columns(chessboard.object_points) > 0);

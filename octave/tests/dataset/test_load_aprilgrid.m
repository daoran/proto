addpath(genpath("proto"));

APRILGRID_CSV = "tests/dataset/1544716471496990204.csv";
[retval, grid] = load_aprilgrid(APRILGRID_CSV);

assert(grid.configured == 1);
assert(grid.tag_rows == 6);
assert(grid.tag_cols == 6);
assert(abs(grid.tag_size - 0.085000) < 1e-10);
assert(abs(grid.tag_spacing - 0.3) < 1e-10);

assert((grid.ts - uint64(1544716471496990204)) == 0);
assert(length(grid.id) > 0);
assert(length(grid.keypoints) > 0);
assert(rows(grid.keypoints) == 2);
assert(columns(grid.keypoints) == length(grid.keypoints));

assert(grid.estimated == 1);
assert(length(grid.points_CF) > 0);
assert(rows(grid.points_CF) == 3);
assert(columns(grid.points_CF) == length(grid.points_CF));

assert(rows(grid.q_CF) == 4);
assert(columns(grid.q_CF) == 1);
assert(rows(grid.r_CF) == 3);
assert(columns(grid.r_CF) == 1);
assert(rows(grid.T_CF) == 4);
assert(columns(grid.T_CF) == 4);

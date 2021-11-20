addpath(genpath("proto"));

grid = {};
grid.rows = 6.0;
grid.cols = 6.0;
grid.size = 0.088;
grid.spacing = 0.3;

[i, j] = aprilgrid_grid_index(grid, 0);
assert(i == 0);
assert(j == 0);

[i, j] = aprilgrid_grid_index(grid, 5);
assert(i == 0);
assert(j == 5);

[i, j] = aprilgrid_grid_index(grid, 7);
assert(i == 1);
assert(j == 1);

[i, j] = aprilgrid_grid_index(grid, 17);
assert(i == 2);
assert(j == 5);

addpath(genpath("proto"));

calib_target = calib_target_init();

assert(calib_target.nb_rows != 0);
assert(calib_target.nb_cols != 0);
assert(calib_target.nb_corners > 0);
assert(calib_target.tag_size == 0.2);
assert(rows(calib_target.object_points) == 3);
assert(columns(calib_target.object_points) > 0);

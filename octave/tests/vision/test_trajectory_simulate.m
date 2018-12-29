addpath(genpath("prototype"));

camera = camera_create([680; 480], 90.0);
chessboard = chessboard_create(nb_rows=4, nb_cols=4, tag_size=0.2);
data = trajectory_simulate(camera, chessboard);

assert(length(data.time) > 0);
assert(isfield(data, "camera"));
assert(isfield(data, "chessboard"));
assert(length(data.T_WC) > 0);
assert(isfield(data, "T_WF"));
assert(length(data.z_data) == length(data.time));
assert(length(data.p_data) == length(data.time));

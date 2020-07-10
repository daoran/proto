addpath(genpath("proto"));

timestamps = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];
positions = rand(3, 6);
orientations = rand(4, 6);
ctraj = ctraj_init(timestamps, positions, orientations)

addpath(genpath("proto"));
graphics_toolkit("fltk");

rpy = deg2rad([10.0; 20.0; 30.0])
C = euler321(rpy)

q = euler2quat(rpy)
rpy_ = quat2euler(q)

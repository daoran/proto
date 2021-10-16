addpath(genpath("proto"));

p_W = [10; rand(2, 1)];
[theta, phi] = point2bearing(p_W)

% m = [cos(phi) * sin(theta), 
%      -sin(phi), cos(phi) *cos(theta)]


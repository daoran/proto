addpath(genpath("proto"));

rot = [1.0; 0.0; 0.0; 0.0];
trans = [0.0; 0.0; 0.0];
data = [rot; trans];
pose = pose_t(0, data);

graph = graph_t();

factor = {};
graph = graph_add_factor(graph, factor);

camera = {};
p = rand(3, 1);
z = rand(2, 1);
covar=eye(2);
factor = ba_factor(camera, p, z, covar);
[r, J] = factor.eval()

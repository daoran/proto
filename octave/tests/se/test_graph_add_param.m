addpath(genpath("proto"));

rot = [1.0; 0.0; 0.0; 0.0];
trans = [0.0; 0.0; 0.0];
param = [rot; trans];
pose = pose_init(0, param);

graph = graph_init();
graph = graph_add_param(graph, pose);

assert(graph.params{1}.type == "pose")
assert(graph.params{1}.ts == 0)
assert(graph.params{1}.param == param)

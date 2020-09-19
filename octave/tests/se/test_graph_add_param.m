addpath(genpath("proto"));

rot = [1.0; 0.0; 0.0; 0.0];
trans = [0.0; 0.0; 0.0];
data = [rot; trans];
pose = pose_t(0, data);

graph = graph_t();
graph = graph_add_param(graph, pose);

assert(graph.params{1}.type_info == "pose_t")
assert(graph.params{1}.ts == 0)
assert(graph.params{1}.data == data)

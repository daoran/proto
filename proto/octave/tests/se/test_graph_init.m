addpath(genpath("proto"));

graph = graph_init();
assert(isfield(graph, "params"));
assert(isfield(graph, "factors"));

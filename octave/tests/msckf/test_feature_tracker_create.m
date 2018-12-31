addpath(genpath("prototype"));

tracker = feature_tracker_create();
assert(isfield(tracker, "T_C0C1"));
assert(isfield(tracker, "T_SC0"));

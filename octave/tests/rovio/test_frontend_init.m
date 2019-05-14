addpath(genpath("prototype"));

frontend = frontend_init();
assert(isfield(frontend, "T_C0C1"));
assert(isfield(frontend, "T_SC0"));

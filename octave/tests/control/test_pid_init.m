addpath(genpath("proto"));

pid = pid_init(0.1, 0.2, 0.3);

assert(isfield(pid, "k_p"));
assert(isfield(pid, "k_i"));
assert(isfield(pid, "k_d"));

assert(pid.k_p == 0.1);
assert(pid.k_i == 0.2);
assert(pid.k_d == 0.3);

assert(isfield(pid, "error_sum"));
assert(isfield(pid, "error_prev"));
assert(isfield(pid, "error_p"));
assert(isfield(pid, "error_i"));
assert(isfield(pid, "error_d"));

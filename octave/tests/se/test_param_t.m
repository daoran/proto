addpath(genpath("proto"));

param = param_t("param_t", 0, [1; 2; 3], 3);

assert(param.type_info == "param_t");
assert(param.ts == 0);
assert(param.data == [1; 2; 3]);
assert(param.min_size == 3);

param.print()

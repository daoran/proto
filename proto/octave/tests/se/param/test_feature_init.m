addpath(genpath("proto"));

# Standard XYZ parameterization
data = [1; 2; 3];
param = feature_init(0, data);

assert(param.type == "feature");
assert(param.id == 0);
assert(param.param == [1; 2; 3]);
assert(param.min_dims == 3);

# Inverse depth parameterization

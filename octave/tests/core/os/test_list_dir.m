addpath(genpath("prototype"));

listings = list_dir(".");
assert(size(listings) > 0);
assert(isfield(listings, "name") == 1);
assert(isfield(listings, "date") == 1);
assert(isfield(listings, "bytes") == 1);
assert(isfield(listings, "isdir") == 1);

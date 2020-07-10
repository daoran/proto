addpath(genpath("proto"));

assert(strcmp(join_paths("/home/chutsu", "test"), "/home/chutsu/test") == 1);
assert(strcmp(join_paths("/home/chutsu", "/test"), "/home/chutsu/test") == 1);
assert(strcmp(join_paths("/home/chutsu/", "test"), "/home/chutsu/test") == 1);
assert(strcmp(join_paths("/home/chutsu/", "/test"), "/home/chutsu/test") == 1);

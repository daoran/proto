addpath(genpath("prototype"));

YAML_FILE = "prototype/core/config/test.yaml";
assert(strcmp(yaml_parse_str(YAML_FILE, "string"), "hello") == 1);
assert(yaml_parse_int(YAML_FILE, "int") == 1);
assert(yaml_parse_dbl(YAML_FILE, "double") == 2.0);
assert(any(yaml_parse_vec(YAML_FILE, "vector") == [1.0, 2.0, 3.0, 4.0, 5.0]) == 1);
assert(any(yaml_parse_mat(YAML_FILE, "matrix") == eye(4)) == 1);

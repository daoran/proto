#include "yaml_config.hpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  std::string yaml_path;
  std::string yaml_key;
  parse_args(nlhs, plhs, nrhs, prhs, yaml_path, yaml_key);

  try {
    config_t config{yaml_path};
    std::string value;
    parse(config, yaml_key, value);
    plhs[0] = mxCreateString(value.c_str());

  } catch (const YAML::Exception &e) {
    mexErrMsgIdAndTxt("proto:error", e.msg.c_str());
  }

  return;
}

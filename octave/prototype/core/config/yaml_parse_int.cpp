#include "yaml_config.hpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  std::string yaml_path;
  std::string yaml_key;
  parse_args(nlhs, plhs, nrhs, prhs, yaml_path, yaml_key);

  try {
    int value = 0;
    config_t config{yaml_path};
    parse(config, yaml_key, value);

    plhs[0] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    int* data = (int*) mxGetData(plhs[0]);
    data[0] = value;
  } catch (const YAML::Exception& e) {
    mexErrMsgIdAndTxt("proto:error", e.msg.c_str());
  }


  return;
}

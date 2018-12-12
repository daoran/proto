#include "yaml_config.hpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  std::string yaml_path;
  std::string yaml_key;
  parse_args(nlhs, plhs, nrhs, prhs, yaml_path, yaml_key);

  config_t config{yaml_path};
  matx_t value;
  parse(config, yaml_key, value);

  const long nb_rows = value.rows();
  const long nb_cols = value.cols();
  plhs[0] = mxCreateNumericMatrix(nb_rows, nb_cols, mxDOUBLE_CLASS, mxREAL);
  double* data = (double*) mxGetData(plhs[0]);

  int el_idx = 0;
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      data[el_idx] = value(i, j);
      el_idx++;
    }
  }

  return;
}

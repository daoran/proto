#include <mex.h>
#include <prototype/prototype.hpp>

/**
 * Check number of arguments
 */
inline void nargchk(bool cond) {
  if (!cond) {
    mexErrMsgIdAndTxt("prototype:error", "Wrong number of arguments!");
  }
}

/**
 * Parse args
 */
void parse_args(int nlhs, mxArray *plhs[],
                int nrhs, const mxArray *prhs[],
                std::string &data_path) {
  UNUSED(plhs);

  // Parse args
  nargchk(nlhs >= 0 && nrhs == 1);
  // -- YAML file path
  std::shared_ptr<char> arg(mxArrayToString(prhs[0]), &free);
  data_path = std::string{arg.get()};
}

mxArray *convert(const proto::timestamps_t &timestamps) {
  const long nb_ts = timestamps.size();
  mxArray *out = mxCreateNumericMatrix(nb_ts, 1, mxUINT64_CLASS, mxREAL);
  uint64_t *ts_data = (uint64_t *) mxGetData(out);
  for (long i = 0; i < nb_ts; i++) {
    ts_data[i] = timestamps[i];
  }

  return out;
}

mxArray *convert(const std::vector<std::string> &string_vector) {
  const int nb_elements = string_vector.size();
  const int dims[2] = {nb_elements, 1};
  mxArray *out = mxCreateCellArray(1, dims);

  for (int i = 0; i < nb_elements; i++) {
    mxSetCell(out, i, mxCreateString(string_vector[i].c_str()));
  }

  return out;
}

mxArray *convert(const proto::matx_t &m) {
  const long nb_rows = m.rows();
  const long nb_cols = m.cols();
  mxArray *out = mxCreateNumericMatrix(nb_rows, nb_cols, mxDOUBLE_CLASS, mxREAL);
  double* data = (double*) mxGetData(out);

  long el_idx = 0;
  for (long i = 0; i < nb_rows; i++) {
    for (long j = 0; j < nb_cols; j++) {
      data[el_idx] = m(i, j);
      el_idx++;
    }
  }

  return out;
}

mxArray *convert(const proto::vec3s_t &m) {
  const long nb_rows = 3;
  const long nb_cols = m.size();
  mxArray *out = mxCreateNumericMatrix(nb_rows, nb_cols, mxDOUBLE_CLASS, mxREAL);
  double* data = (double*) mxGetData(out);

  long el_idx = 0;
  for (size_t k = 0; k < m.size(); k++) {
    const auto &element = m[k];

    for (long i = 0; i < element.rows(); i++) {
      for (long j = 0; j < element.cols(); j++) {
        data[el_idx] = element(i, j);
        el_idx++;
      }
    }
  }

  return out;
}

mxArray *convert(const int &val) {
  mxArray *out = mxCreateNumericMatrix(1, 1, mxINT64_CLASS, mxREAL);
  int* data = (int*) mxGetData(out);
  data[0] = val;
  return out;
}

mxArray *convert(const double &val) {
  mxArray *out = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
  double* data = (double*) mxGetData(out);
  data[0] = val;
  return out;
}

mxArray *convert(const std::string &s) {
  return mxCreateString(s.c_str());
}

mxArray *convert_imu_data(const proto::euroc_imu_t &imu_data) {
  const char *keys[] = {
    // Data
    "data_dir",
    "timestamps",
    "w_B",
    "a_B",
    // Sensor properties
    "sensor_type",
    "comment",
    "T_BS",
    "rate_hz",
    "gyro_noise_density",
    "gyro_random_walk",
    "accel_noise_density",
    "accel_random_walk"
  };

  mxArray *data = mxCreateStructMatrix(1, 1, 12, keys);
  mxSetField(data, 0, keys[0], convert(imu_data.data_dir.c_str()));
  mxSetField(data, 0, keys[1], convert(imu_data.timestamps));
  mxSetField(data, 0, keys[2], convert(imu_data.w_B));
  mxSetField(data, 0, keys[3], convert(imu_data.a_B));
  mxSetField(data, 0, keys[4], convert(imu_data.sensor_type.c_str()));
  mxSetField(data, 0, keys[5], convert(imu_data.comment.c_str()));
  mxSetField(data, 0, keys[6], convert(imu_data.T_BS));
  mxSetField(data, 0, keys[7], convert(imu_data.rate_hz));
  mxSetField(data, 0, keys[8], convert(imu_data.gyro_noise_density));
  mxSetField(data, 0, keys[9], convert(imu_data.gyro_random_walk));
  mxSetField(data, 0, keys[10], convert(imu_data.accel_noise_density));
  mxSetField(data, 0, keys[11], convert(imu_data.accel_random_walk));

  return data;
}

mxArray *convert_camera_data(const proto::euroc_camera_t &cam_data) {
  const char *keys[] = {
    // Data
    "data_dir",
    "timestamps",
    "image_paths",
    // Sensor properties
    "sensor_type",
    "comment",
    "T_BS",
    "rate_hz",
    "resolution",
    "camera_model",
    "intrinsics",
    "distortion_model",
    "distortion",
  };

  mxArray *data = mxCreateStructMatrix(1, 1, 12, keys);
  mxSetField(data, 0, keys[0], convert(cam_data.data_dir.c_str()));
  mxSetField(data, 0, keys[1], convert(cam_data.timestamps));
  mxSetField(data, 0, keys[2], convert(cam_data.image_paths));
  mxSetField(data, 0, keys[3], convert(cam_data.sensor_type.c_str()));
  mxSetField(data, 0, keys[4], convert(cam_data.comment.c_str()));
  mxSetField(data, 0, keys[5], convert(cam_data.T_BS));
  mxSetField(data, 0, keys[6], convert(cam_data.rate_hz));
  mxSetField(data, 0, keys[7], convert(cam_data.resolution));
  mxSetField(data, 0, keys[8], convert(cam_data.camera_model.c_str()));
  mxSetField(data, 0, keys[9], convert(cam_data.intrinsics));
  mxSetField(data, 0, keys[10], convert(cam_data.distortion_model.c_str()));
  mxSetField(data, 0, keys[11], convert(cam_data.distortion_coefficients));

  return data;
}

/**
 * MEX-entry point
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // Parse args
  std::string data_path;
  parse_args(nlhs, plhs, nrhs, prhs, data_path);

  // Load EuRoC dataset
  proto::euroc_data_t data(data_path);
  if (data.ok == false) {
    mexErrMsgIdAndTxt("prototype:error", "Failed to load EuRoC dataset!");
  }

  // Convert to octave struct
  mxArray *imu0 = convert_imu_data(data.imu_data);
  mxArray *cam0 = convert_camera_data(data.cam0_data);
  mxArray *cam1 = convert_camera_data(data.cam1_data);

  const char *keys[] = {"imu0", "cam0", "cam1"};
  mxArray *euroc_data = mxCreateStructMatrix(1, 1, 3, keys);
  mxSetField(euroc_data, 0, keys[0], imu0);
  mxSetField(euroc_data, 0, keys[1], cam0);
  mxSetField(euroc_data, 0, keys[2], cam1);

  plhs[0] = euroc_data;
}

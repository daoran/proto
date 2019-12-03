#include <mex.h>
#include <proto/proto.hpp>

/**
 * Check number of arguments
 */
inline void nargchk(bool cond) {
  if (!cond) {
    mexErrMsgIdAndTxt("proto:error", "Wrong number of arguments!");
  }
}

/**
 * Parse args
 */
void parse_args(int nlhs,
                mxArray *plhs[],
                int nrhs,
                const mxArray *prhs[],
                std::string &data_path,
                std::string &date,
                std::string &seq) {
  UNUSED(plhs);

  // Parse args
  nargchk(nlhs >= 0 && nrhs == 3);

  // -- KITTI RAW data base path
  std::shared_ptr<char> arg0(mxArrayToString(prhs[0]), &free);
  data_path = std::string{arg0.get()};

  // -- Date
  std::shared_ptr<char> arg1(mxArrayToString(prhs[1]), &free);
  date = std::string{arg1.get()};

  // -- Sequenece
  std::shared_ptr<char> arg2(mxArrayToString(prhs[2]), &free);
  seq = std::string{arg2.get()};
}

/**
 * Convert timestamps
 */
mxArray *convert(const proto::timestamps_t &timestamps) {
  const long nb_ts = timestamps.size();
  mxArray *out = mxCreateNumericMatrix(nb_ts, 1, mxUINT64_CLASS, mxREAL);
  uint64_t *ts_data = (uint64_t *) mxGetData(out);
  for (long i = 0; i < nb_ts; i++) {
    ts_data[i] = timestamps[i];
  }

  return out;
}

/**
 * Convert vector of strings
 */
mxArray *convert(const std::vector<std::string> &string_vector) {
  const int nb_elements = string_vector.size();
  const int dims[2] = {nb_elements, 1};
  mxArray *out = mxCreateCellArray(1, dims);

  for (int i = 0; i < nb_elements; i++) {
    mxSetCell(out, i, mxCreateString(string_vector[i].c_str()));
  }

  return out;
}

/**
 * Convert vector of long
 */
mxArray *convert(const std::vector<long> &long_vector) {
  const long nb_ts = long_vector.size();
  mxArray *out = mxCreateNumericMatrix(nb_ts, 1, mxINT64_CLASS, mxREAL);
  uint64_t *data = (uint64_t *) mxGetData(out);
  for (long i = 0; i < nb_ts; i++) {
    data[i] = long_vector[i];
  }

  return out;
}

/**
 * Convert vector of double
 */
mxArray *convert(const std::vector<double> &double_vector) {
  const long nb_ts = double_vector.size();
  mxArray *out = mxCreateNumericMatrix(nb_ts, 1, mxDOUBLE_CLASS, mxREAL);
  double_t *data = (double *) mxGetData(out);
  for (long i = 0; i < nb_ts; i++) {
    data[i] = double_vector[i];
  }

  return out;
}

/**
 * Convert array of vec2_t
 */
mxArray *convert(const std::array<proto::vec2_t, 4> &vec2_array) {
  const int nb_elements = vec2_array.size();
  const int dims[2] = {nb_elements, 1};
  mxArray *out = mxCreateCellArray(1, dims);

  for (int i = 0; i < nb_elements; i++) {
    mxArray *tmp = mxCreateNumericMatrix(2, 1, mxDOUBLE_CLASS, mxREAL);
    double *data = (double *) mxGetData(tmp);
    data[0] = vec2_array[i](0);
    data[1] = vec2_array[i](1);
    mxSetCell(out, i, tmp);
  }

  return out;
}

/**
 * Convert array of vec3_t
 */
mxArray *convert(const std::array<proto::vec3_t, 4> &vec3_array) {
  const int nb_elements = vec3_array.size();
  const int dims[2] = {nb_elements, 1};
  mxArray *out = mxCreateCellArray(1, dims);

  for (int i = 0; i < nb_elements; i++) {
    mxArray *tmp = mxCreateNumericMatrix(3, 1, mxDOUBLE_CLASS, mxREAL);
    double *data = (double *) mxGetData(tmp);
    data[0] = vec3_array[i](0);
    data[1] = vec3_array[i](1);
    data[2] = vec3_array[i](2);
    mxSetCell(out, i, tmp);
  }

  return out;
}

/**
 * Convert array of vecx_t
 */
mxArray *convert(const std::array<proto::vecx_t, 4> &vecx_array) {
  const int nb_elements = vecx_array.size();
  const int dims[2] = {nb_elements, 1};
  mxArray *out = mxCreateCellArray(1, dims);

  for (int i = 0; i < nb_elements; i++) {
    const proto::vecx_t &vec = vecx_array[i];
    mxArray *tmp = mxCreateNumericMatrix(vec.size(), 1, mxDOUBLE_CLASS, mxREAL);
    double *data = (double *) mxGetData(tmp);
    for (int j = 0; j < vec.size(); j++) {
      data[j] = vec(j);
    }

    mxSetCell(out, i, tmp);
  }

  return out;
}

/**
 * Convert array of mat3_t
 */
mxArray *convert(const std::array<proto::mat3_t, 4> &mat3_array) {
  const int nb_elements = mat3_array.size();
  const int dims[2] = {nb_elements, 1};
  mxArray *out = mxCreateCellArray(1, dims);

  for (int i = 0; i < nb_elements; i++) {
    mxArray *tmp = mxCreateNumericMatrix(3, 3, mxDOUBLE_CLASS, mxREAL);
    double *data = (double *) mxGetData(tmp);
    data[0] = mat3_array[i](0, 0);
    data[1] = mat3_array[i](0, 1);
    data[2] = mat3_array[i](0, 2);

    data[3] = mat3_array[i](1, 0);
    data[4] = mat3_array[i](1, 1);
    data[5] = mat3_array[i](1, 2);

    data[6] = mat3_array[i](2, 0);
    data[7] = mat3_array[i](2, 1);
    data[8] = mat3_array[i](2, 2);

    mxSetCell(out, i, tmp);
  }

  return out;
}

/**
 * Convert array of mat34_t
 */
mxArray *convert(const std::array<proto::mat34_t, 4> &mat34_array) {
  const int nb_elements = mat34_array.size();
  const int dims[2] = {nb_elements, 1};
  mxArray *out = mxCreateCellArray(1, dims);

  for (int i = 0; i < nb_elements; i++) {
    mxArray *tmp = mxCreateNumericMatrix(3, 4, mxDOUBLE_CLASS, mxREAL);
    double *data = (double *) mxGetData(tmp);
    data[0] = mat34_array[i](0, 0);
    data[1] = mat34_array[i](0, 1);
    data[2] = mat34_array[i](0, 2);
    data[3] = mat34_array[i](0, 3);

    data[4] = mat34_array[i](1, 0);
    data[5] = mat34_array[i](1, 1);
    data[6] = mat34_array[i](1, 2);
    data[7] = mat34_array[i](1, 3);

    data[8] = mat34_array[i](2, 0);
    data[9] = mat34_array[i](2, 1);
    data[10] = mat34_array[i](2, 2);
    data[11] = mat34_array[i](2, 3);

    mxSetCell(out, i, tmp);
  }

  return out;
}

/**
 * Convert matrix
 */
mxArray *convert(const proto::matx_t &m) {
  const long nb_rows = m.rows();
  const long nb_cols = m.cols();
  mxArray *out =
      mxCreateNumericMatrix(nb_rows, nb_cols, mxDOUBLE_CLASS, mxREAL);
  double *data = (double *) mxGetData(out);

  long el_idx = 0;
  for (long i = 0; i < nb_rows; i++) {
    for (long j = 0; j < nb_cols; j++) {
      data[el_idx] = m(i, j);
      el_idx++;
    }
  }

  return out;
}

/**
 * Convert a list of column vectors of size 3
 */
mxArray *convert(const proto::vec3s_t &m) {
  const long nb_rows = 3;
  const long nb_cols = m.size();
  mxArray *out =
      mxCreateNumericMatrix(nb_rows, nb_cols, mxDOUBLE_CLASS, mxREAL);
  double *data = (double *) mxGetData(out);

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

/**
 * Convert integer
 */
mxArray *convert(const int &val) {
  mxArray *out = mxCreateNumericMatrix(1, 1, mxINT64_CLASS, mxREAL);
  int *data = (int *) mxGetData(out);
  data[0] = val;
  return out;
}

/**
 * Convert double
 */
mxArray *convert(const double &val) {
  mxArray *out = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
  double *data = (double *) mxGetData(out);
  data[0] = val;
  return out;
}

/**
 * Convert string
 */
mxArray *convert(const std::string &s) { return mxCreateString(s.c_str()); }

/**
 * Convert cam2cam_t to octave-struct
 */
mxArray *convert_cam2cam_data(const proto::calib_cam2cam_t &cam2cam) {
  const char *keys[] = {"file_path",
                        "calib_time",
                        "corner_dist",
                        "S",
                        "K",
                        "D",
                        "R",
                        "T",
                        "S_rect",
                        "R_rect",
                        "P_rect"};
  mxArray *data = mxCreateStructMatrix(1, 1, 11, keys);
  mxSetField(data, 0, keys[0], convert(cam2cam.file_path.c_str()));
  mxSetField(data, 0, keys[1], convert(cam2cam.calib_time.c_str()));
  mxSetField(data, 0, keys[2], convert(cam2cam.corner_dist));
  mxSetField(data, 0, keys[3], convert(cam2cam.S));
  mxSetField(data, 0, keys[4], convert(cam2cam.K));
  mxSetField(data, 0, keys[5], convert(cam2cam.D));
  mxSetField(data, 0, keys[6], convert(cam2cam.R));
  mxSetField(data, 0, keys[7], convert(cam2cam.T));
  mxSetField(data, 0, keys[8], convert(cam2cam.S_rect));
  mxSetField(data, 0, keys[9], convert(cam2cam.R_rect));
  mxSetField(data, 0, keys[10], convert(cam2cam.P_rect));

  return data;
}

/**
 * Convert imu2velo_t to octave-struct
 */
mxArray *convert_imu2velo_data(const proto::calib_imu2velo_t &imu2velo) {
  const char *keys[] = {"file_path", "calib_time", "R", "t", "T_velo_imu"};
  mxArray *data = mxCreateStructMatrix(1, 1, 5, keys);
  mxSetField(data, 0, keys[0], convert(imu2velo.file_path.c_str()));
  mxSetField(data, 0, keys[1], convert(imu2velo.calib_time.c_str()));
  mxSetField(data, 0, keys[2], convert(imu2velo.R));
  mxSetField(data, 0, keys[3], convert(imu2velo.t));
  mxSetField(data, 0, keys[4], convert(imu2velo.T_velo_imu));

  return data;
}

/**
 * Convert velo2cam_t to octave-struct
 */
mxArray *convert_velo2cam_data(const proto::calib_velo2cam_t &velo2cam) {
  const char *keys[] =
      {"file_path", "calib_time", "R", "t", "df", "dc", "T_cam_velo"};
  mxArray *data = mxCreateStructMatrix(1, 1, 7, keys);
  mxSetField(data, 0, keys[0], convert(velo2cam.file_path.c_str()));
  mxSetField(data, 0, keys[1], convert(velo2cam.calib_time.c_str()));
  mxSetField(data, 0, keys[2], convert(velo2cam.R));
  mxSetField(data, 0, keys[3], convert(velo2cam.t));
  mxSetField(data, 0, keys[4], convert(velo2cam.df));
  mxSetField(data, 0, keys[5], convert(velo2cam.dc));
  mxSetField(data, 0, keys[6], convert(velo2cam.T_cam_velo));

  return data;
}

/**
 * Convert oxts_t to octave-struct
 */
mxArray *convert_oxts_data(const proto::oxts_t &oxts) {
  const char *keys[] = {"oxts_dir",
                        "timestamps",
                        "time",
                        "gps",
                        "rpy",
                        "p_G",
                        "v_G",
                        "v_B",
                        "a_G",
                        "a_B",
                        "w_G",
                        "w_B",
                        "pos_accuracy",
                        "vel_accuracy"};
  mxArray *data = mxCreateStructMatrix(1, 1, 14, keys);
  mxSetField(data, 0, keys[0], convert(oxts.oxts_dir.c_str()));
  mxSetField(data, 0, keys[1], convert(oxts.timestamps));
  mxSetField(data, 0, keys[2], convert(oxts.time));
  mxSetField(data, 0, keys[3], convert(oxts.gps));
  mxSetField(data, 0, keys[4], convert(oxts.rpy));
  mxSetField(data, 0, keys[5], convert(oxts.p_G));
  mxSetField(data, 0, keys[6], convert(oxts.v_G));
  mxSetField(data, 0, keys[7], convert(oxts.v_B));
  mxSetField(data, 0, keys[8], convert(oxts.a_G));
  mxSetField(data, 0, keys[9], convert(oxts.a_B));
  mxSetField(data, 0, keys[10], convert(oxts.w_G));
  mxSetField(data, 0, keys[11], convert(oxts.w_B));
  mxSetField(data, 0, keys[12], convert(oxts.pos_accuracy));
  mxSetField(data, 0, keys[13], convert(oxts.vel_accuracy));

  return data;
}

/**
 * Convert kitti_raw_t to octave-struct
 */
mxArray *convert_kitti_raw_data(const proto::kitti_raw_t &kitti_raw) {
  const char *keys[] = {"raw_dir",
                        "date",
                        "seq",
                        "date_dir",
                        "drive_dir",
                        "cam2cam",
                        "imu2velo",
                        "velo2cam",
                        "oxts",
                        "cam0",
                        "cam1",
                        "cam2",
                        "cam3"};

  mxArray *data = mxCreateStructMatrix(1, 1, 13, keys);
  mxSetField(data, 0, keys[0], convert(kitti_raw.raw_dir.c_str()));
  mxSetField(data, 0, keys[1], convert(kitti_raw.date.c_str()));
  mxSetField(data, 0, keys[2], convert(kitti_raw.seq.c_str()));
  mxSetField(data, 0, keys[3], convert(kitti_raw.date_dir.c_str()));
  mxSetField(data, 0, keys[4], convert(kitti_raw.drive_dir.c_str()));
  mxSetField(data, 0, keys[5], convert_cam2cam_data(kitti_raw.cam2cam));
  mxSetField(data, 0, keys[6], convert_imu2velo_data(kitti_raw.imu2velo));
  mxSetField(data, 0, keys[7], convert_velo2cam_data(kitti_raw.velo2cam));
  mxSetField(data, 0, keys[8], convert_oxts_data(kitti_raw.oxts));
  mxSetField(data, 0, keys[9], convert(kitti_raw.cam0));
  mxSetField(data, 0, keys[10], convert(kitti_raw.cam1));
  mxSetField(data, 0, keys[11], convert(kitti_raw.cam2));
  mxSetField(data, 0, keys[12], convert(kitti_raw.cam3));

  return data;
}

/**
 * MEX-entry point
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // Parse args
  std::string data_path;
  std::string date;
  std::string seq;
  parse_args(nlhs, plhs, nrhs, prhs, data_path, date, seq);

  // Load EuRoC dataset
  proto::kitti_raw_t data(data_path, date, seq);
  if (proto::kitti_raw_load(data) != 0) {
    mexErrMsgIdAndTxt("proto:error", "Failed to load KITTI raw dataset!");
  }

  // Return
  plhs[0] = convert_kitti_raw_data(data);
}

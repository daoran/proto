/**
 * Hello World
 */

namespace prototype {

/**
 * Calibration target.
 */
struct calib_target_t {
  std::string target_type;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  calib_target_t();
  ~calib_target_t();
};

/**
 * Load calibration target.
 * @returns 0 or -1 for success or failure
 */
int calib_target_load(calib_target_t &ct, const std::string &target_file);

} // namespace prototype

#include "prototype/dataset/euroc/ground_truth.hpp"

namespace prototype {

int ground_truth_load(ground_truth_t &gtd) {
  // Load ground truth data
  const std::string gnd_data_path = gtd.data_dir + "/data.csv";
  matx_t data;
  if (csv2mat(gnd_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load ground truth data [%s]!", gnd_data_path.c_str());
    return -1;
  }

  const double t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    const long ts = data(i, 0);
    gtd.timestamps.push_back(ts);
    gtd.time.push_back(((double) ts - t0) * 1e-9);
    gtd.p_RS_R.emplace_back(data(i, 1), data(i, 2), data(i, 3));
    gtd.q_RS.emplace_back(data(i, 5), data(i, 6), data(i, 7), data(i, 4));
    gtd.v_RS_R.emplace_back(data(i, 8), data(i, 9), data(i, 10));
    gtd.b_w_RS_S.emplace_back(data(i, 11), data(i, 12), data(i, 13));
    gtd.b_a_RS_S.emplace_back(data(i, 14), data(i, 15), data(i, 16));
  }

  return 0;
}

} //  namespace prototype

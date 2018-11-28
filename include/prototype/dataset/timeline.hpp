#ifndef PROTOTYPE_DATASET_TIMELINE_HPP
#define PROTOTYPE_DATASET_TIMELINE_HPP

#include "prototype/core/core.hpp"

namespace prototype {

/* Timeline type */
#define NOT_SET 0
#define IMU_EVENT 1
#define CAMERA_EVENT 2

struct timeline_event_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // General
  int type = 0;
  long ts = 0.0;

  // IMU data
  vec3_t a_m = zeros(3, 1);
  vec3_t w_m = zeros(3, 1);

  // Camera data
  int camera_index = 0;
  std::string image_path;

  timeline_event_t();
  timeline_event_t(const long ts_, const vec3_t &a_m_, const vec3_t &w_m_);
  timeline_event_t(const long ts_,
                   const int camera_index_,
                   const std::string &image_path_);
  ~timeline_event_t();
};

struct timeline_t {
  std::multimap<long, timeline_event_t> data;
  std::set<long> timestamps;

  timeline_t();
  ~timeline_t();
};

} //  namespace prototype
#endif // PROTOTYPE_DATASET_TIMELINE_HPP

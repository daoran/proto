#ifndef PROTO_BA_HPP
#define PROTO_BA_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "proto/core/core.hpp"
#include "proto/estimation/factor.hpp"

namespace proto {

struct ba_data_t {
  mat3_t cam_K;

  poses_t cam_poses;
  pose_t target_pose;
  int nb_frames;

  std::vector<keypoints_t> keypoints;
  int **point_ids;
  int nb_ids;

  real_t **points;
  int nb_points;

  ba_data_t(const std::string &data_path);
  ~ba_data_t();
};

int ba_residual_size(ba_data_t &data);
vecx_t ba_residuals(ba_data_t &data);
matx_t ba_jacobian(ba_data_t &data);
void ba_update(ba_data_t &data, const vecx_t &e, const matx_t &E);
real_t ba_cost(const vecx_t &e);
void ba_solve(ba_data_t &data);

} // namespace proto
#endif // PROTO_BA_HPP

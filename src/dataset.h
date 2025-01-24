#pragma once

#include "data.h"
#include "state_estimation.h"

pose_t *load_poses(const char *fp, int *num_poses);
int **assoc_pose_data(pose_t *gnd_poses,
                      size_t num_gnd_poses,
                      pose_t *est_poses,
                      size_t num_est_poses,
                      double threshold,
                      size_t *num_matches);

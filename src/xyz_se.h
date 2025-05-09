#pragma once

#include "xyz.h"

// ///////////////////////
// // INERTIAL ODOMETRY //
// ///////////////////////
//
// typedef struct inertial_odometry_t {
//   // IMU Parameters
//   imu_params_t imu_params;
//
//   // Factors
//   int num_factors;
//   imu_factor_t *factors;
//   marg_factor_t *marg;
//
//   // Variables
//   pose_t *poses;
//   velocity_t *vels;
//   imu_biases_t *biases;
// } inertial_odometry_t;
//
// inertial_odometry_t *inertial_odometry_malloc(void);
// void inertial_odometry_free(inertial_odometry_t *odom);
// void inertial_odometry_save(const inertial_odometry_t *odom,
//                             const char *save_path);
// param_order_t *inertial_odometry_param_order(const void *data,
//                                              int *sv_size,
//                                              int *r_size);
// void inertial_odometry_cost(const void *data, real_t *r);
// void inertial_odometry_linearize_compact(const void *data,
//                                          const int sv_size,
//                                          param_order_t *hash,
//                                          real_t *H,
//                                          real_t *g,
//                                          real_t *r);

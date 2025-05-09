#include "xyz_se.h"

// ///////////////////////
// // INERTIAL ODOMETRY //
// ///////////////////////
//
// /**
//  * Malloc inertial odometry.
//  */
// inertial_odometry_t *inertial_odometry_malloc(void) {
//   inertial_odometry_t *io = malloc(sizeof(inertial_odometry_t) * 1);
//
//   io->num_factors = 0;
//   io->factors = NULL;
//   io->marg = NULL;
//
//   io->poses = NULL;
//   io->vels = NULL;
//   io->biases = NULL;
//
//   return io;
// }
//
// /**
//  * Free inertial odometry.
//  */
// void inertial_odometry_free(inertial_odometry_t *odom) {
//   free(odom->factors);
//   free(odom->poses);
//   free(odom->vels);
//   free(odom->biases);
//   free(odom);
// }
//
// /**
//  * Save inertial odometry.
//  */
// void inertial_odometry_save(const inertial_odometry_t *odom,
//                             const char *save_path) {
//   // Load file
//   FILE *fp = fopen(save_path, "w");
//   if (fp == NULL) {
//     FATAL("Failed to open [%s]!\n", save_path);
//   }
//
//   // Write header
//   fprintf(fp, "#ts,");
//   fprintf(fp, "rx,ry,rz,qw,qx,qy,qz,");
//   fprintf(fp, "vx,vy,vz,");
//   fprintf(fp, "ba_x,ba_y,ba_z,");
//   fprintf(fp, "bg_x,bg_y,bg_z\n");
//
//   // Write data
//   for (int k = 0; k < (odom->num_factors + 1); k++) {
//     const real_t *pos = odom->poses[k].data;
//     const real_t *quat = odom->poses[k].data + 3;
//     const real_t *vel = odom->vels[k].data;
//     const real_t *ba = odom->biases[k].data;
//     const real_t *bg = odom->biases[k].data + 3;
//     fprintf(fp, "%ld,", odom->poses[k].ts);
//     fprintf(fp, "%f,%f,%f,", pos[0], pos[1], pos[2]);
//     fprintf(fp, "%f,%f,%f,%f,", quat[0], quat[1], quat[2], quat[3]);
//     fprintf(fp, "%f,%f,%f,", vel[0], vel[1], vel[2]);
//     fprintf(fp, "%f,%f,%f,", ba[0], ba[1], ba[2]);
//     fprintf(fp, "%f,%f,%f", bg[0], bg[1], bg[2]);
//     fprintf(fp, "\n");
//   }
// }
//
// /**
//  * Determine inertial odometry parameter order.
//  */
// param_order_t *inertial_odometry_param_order(const void *data,
//                                              int *sv_size,
//                                              int *r_size) {
//   // Setup parameter order
//   inertial_odometry_t *odom = (inertial_odometry_t *) data;
//   param_order_t *hash = NULL;
//   int col_idx = 0;
//
//   for (int k = 0; k <= odom->num_factors; k++) {
//     param_order_add_pose(&hash, &odom->poses[k], &col_idx);
//     param_order_add_velocity(&hash, &odom->vels[k], &col_idx);
//     param_order_add_imu_biases(&hash, &odom->biases[k], &col_idx);
//   }
//
//   *sv_size = col_idx;
//   *r_size = odom->num_factors * 15;
//   return hash;
// }
//
// /**
//  * Calculate inertial odometry cost.
//  */
// void inertial_odometry_cost(const void *data, real_t *r) {
//   // Evaluate factors
//   inertial_odometry_t *odom = (inertial_odometry_t *) data;
//   for (int k = 0; k < odom->num_factors; k++) {
//     imu_factor_t *factor = &odom->factors[k];
//     imu_factor_eval(factor);
//     vec_copy(factor->r, factor->r_size, &r[k * factor->r_size]);
//   }
// }
//
// /**
//  * Linearize inertial odometry problem.
//  */
// void inertial_odometry_linearize_compact(const void *data,
//                                          const int sv_size,
//                                          param_order_t *hash,
//                                          real_t *H,
//                                          real_t *g,
//                                          real_t *r) {
//   // Evaluate factors
//   inertial_odometry_t *odom = (inertial_odometry_t *) data;
//
//   for (int k = 0; k < odom->num_factors; k++) {
//     imu_factor_t *factor = &odom->factors[k];
//     imu_factor_eval(factor);
//     vec_copy(factor->r, factor->r_size, &r[k * factor->r_size]);
//
//     solver_fill_hessian(hash,
//                         factor->num_params,
//                         factor->params,
//                         factor->jacs,
//                         factor->r,
//                         factor->r_size,
//                         sv_size,
//                         H,
//                         g);
//   }
// }

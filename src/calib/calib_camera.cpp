#include "prototype/calib/calib_camera.hpp"

namespace prototype {

// int calib_camera_solve(const std::vector<aprilgrid_t> &aprilgrids,
//                        pinhole_t &pinhole,
//                        radtan4_t &radtan) {
//   // Optimization variables
//   double intrinsics[4] = {pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy};
//   double distortion[4] = {radtan.k1, radtan.k2, radtan.p1, radtan.p2};
//   std::vector<double *> points;
//
//   // Setup optimization problem
//   ceres::Problem problem;
//   for (const auto &aprilgrid: aprilgrids) {
//
//     size_t nb_measurements = aprilgrid.ids.size();
//     for (size_t i = 0; i < nb_measurements; i++) {
//       const auto kp = aprilgrid.keypoints[i];
//       const auto p = aprilgrid.points_CF[i];
//       points.push_back(vec2array(p));
//
//       // Form residual and cost function
//       const auto residual = new pinhole_radtan4_residual_t{kp};
//       const auto cost_func =
//         new ceres::AutoDiffCostFunction<pinhole_radtan4_residual_t, // Residual type
//                                         2, // Size of: residual
//                                         4, // Size of: intrinsics
//                                         4, // Size of: distortion
//                                         3  // Size of: point
//                                         >(residual);
//
//       // Add residual block
//       problem.AddResidualBlock(cost_func, // Cost function
//                                NULL,      // Loss function
//                                intrinsics,
//                                distortion,
//                                points[i]);
//     }
//   }
//
//   // Set solver options
//   ceres::Solver::Options options;
//   options.minimizer_progress_to_stdout = true;
//   options.max_num_iterations = 1000;
//
//   // Solve
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   std::cout << summary.FullReport() << std::endl;
//
//   // Show results
//   std::cout << "Optimized intrinsics and distortions:" <<  std::endl;
//   std::cout << "fx: " << intrinsics[0] << std::endl;
//   std::cout << "fy: " << intrinsics[1] << std::endl;
//   std::cout << "cx: " << intrinsics[2] << std::endl;
//   std::cout << "cy: " << intrinsics[3] << std::endl;
//   std::cout << std::endl;
//
//   std::cout << "k1: " << distortion[0] << std::endl;
//   std::cout << "k2: " << distortion[1] << std::endl;
//   std::cout << "p1: " << distortion[2] << std::endl;
//   std::cout << "p2: " << distortion[3] << std::endl;
//   std::cout << std::endl;
//
//   // Free points
//   for (size_t i = 0; i < points.size(); i++) {
//     free(points[i]);
//   }
//
//   // Map results back to pinhole and radtan
//   pinhole.fx = intrinsics[0];
//   pinhole.fy = intrinsics[1];
//   pinhole.cx = intrinsics[2];
//   pinhole.cy = intrinsics[3];
//
//   radtan.k1 = distortion[0];
//   radtan.k2 = distortion[1];
//   radtan.p1 = distortion[2];
//   radtan.p2 = distortion[3];
//
//   return 0;
// }

} //  namespace prototype

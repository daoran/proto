#include <unistd.h>

#include "munit.hpp"
#include "core.hpp"
#include "aprilgrid.hpp"
#include "factor.hpp"

namespace proto {

struct calib_target_t {
	int tag_rows = 6;
	int tag_cols= 6;
	double tag_size = 0.088;
	// double tag_size = 0.021;
	double tag_spacing = 0.3;
};

mat4_t calib_target_origin(const calib_target_t &target,
                           const vec2_t &cam_res,
                           const double hfov) {
	const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_size = target.tag_size;
  const double tag_spacing = target.tag_spacing;

  // Standard pinhole camera model equation
  // x = f * X / Z
  // x / f * X = 1 / Z
  // Z = f * X / x

  // Calculate target center
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double width = tag_cols * tag_size + spacing_x;
  const double height = tag_rows * tag_size + spacing_y;
  const vec2_t center{width / 2.0, height / 2.0};

  // Calculate distance away from target center
  const double half_width =  width / 2.0;
  const double half_resolution_x = cam_res(0) / 2.0;
  const double scale = 0.5; // Target width to be 50% of image space at T_TO
  const double image_width = cam_res(0);
  const auto fx = pinhole_focal(image_width, hfov);
  const auto z_FO = fx * half_width / (half_resolution_x * scale);

  // Form transform of calibration origin (O) wrt fiducial target (F) T_FO
  const mat3_t C_FO = I(3);
  const vec3_t r_FO{center(0), center(1), z_FO};
  const mat4_t T_FO = tf(C_FO, r_FO);

  return T_FO;
}

mat4s_t generate_initial_poses(const calib_target_t &target) {
  // Calculate target width and height
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
  const double calib_depth = calib_width * 1.2;

  // Generate intial poses
  mat4s_t poses;
  // -- First pose is right infront of calibration target
  const vec3_t calib_start{calib_width / 2.0, calib_height / 2.0, calib_depth};
  poses.push_back(lookat(calib_start, calib_center));
  // -- Other 4 are the 4 corners of the aprilgrid
  for (const auto &x : linspace(0.0, calib_width * 1.5, 2)) {
    for (const auto &y : linspace(0.0, calib_height * 1.5, 2)) {
      vec3_t cam_pos{x, y, calib_depth};
      poses.push_back(lookat(cam_pos - (0.5 * calib_center), calib_center));
    }
  }

  return poses;
}

mat4s_t generate_nbv_poses(const calib_target_t &target) {
  // const vec2_t cam_res{640, 480};
  // const double hfov = 90.0;
  // mat4_t T_FO = calib_target_origin(target, cam_res, hfov);

  // Calculate target width and height
  const double tag_rows = target.tag_rows;
  const double tag_cols = target.tag_cols;
  const double tag_spacing = target.tag_spacing;
  const double tag_size = target.tag_size;
  const double spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  const double spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  const double calib_width = tag_cols * tag_size + spacing_x;
  const double calib_height = tag_rows * tag_size + spacing_y;
  const vec3_t calib_center{calib_width / 2.0, calib_height / 2.0, 0.0};
  const double calib_depth = calib_width * 1.2;

  mat4s_t nbv_poses;
  for (const auto &x : linspace(0.0, calib_width * 1.5, 5)) {
    for (const auto &y : linspace(0.0, calib_height * 1.5, 5)) {
      for (const auto &z : linspace(calib_depth, calib_depth * 1.5, 5)) {
        vec3_t cam_pos{x, y, z};
        nbv_poses.push_back(lookat(cam_pos - (0.5 * calib_center), calib_center));
      }
    }
  }

  return nbv_poses;
}

// struct calib_mono_initializer_t {
//   const calib_target_t target;
//   const aprilgrid_detector_t detector;
//   const pinhole_radtan4_t camera_model;
//   const mat4s_t poses_init;
//   size_t pose_idx;
//   std::vector<cv::Mat> frames_init;
//
//   calib_mono_initializer_t(const calib_target_t &target_,
//                            const aprilgrid_detector_t &detector_,
//                            const pinhole_radtan4_t &camera_model_)
//     : target{target_}, detector{detector_}, camera_model{camera_model_},
//       poses_init{generate_initial_poses(target)},
//       pose_idx{0} {}
//
//   int update(const cv::Mat &frame) {
//     cv::Mat frame_vis = frame.clone();
//
//     // Detect AprilGrid
//     aprilgrid_t grid;
//     aprilgrid_set_properties(grid,
//                              target.tag_rows,
//                              target.tag_cols,
//                              target.tag_size,
//                              target.tag_spacing);
//     const auto K = camera_model.K();
//     const auto D = camera_model.dist_params();
//     aprilgrid_detect(grid, detector, frame, K, D);
// 		aprilgrid_draw(grid, frame_vis);
//
//     // Calculate calibration target from camera view
//     vec3s_t object_points;
//     aprilgrid_object_points(grid, object_points);
//     const size_t nb_pts = object_points.size();
//     const matx_t hp_F = vecs2mat(object_points);
//     const mat4_t T_FC = poses_init[pose_idx];
//     const mat4_t T_CF = T_FC.inverse();
//     const matx_t hp_C = T_CF * hp_F;
//     const matx_t p_C = hp_C.block(0, 0, 3, nb_pts);
//
//     // Project target corners to camera frame
//     for (size_t i = 0; i < nb_pts; i++) {
//       const vec3_t p = p_C.block(0, i, 3, 1);
//       const vec3_t pt{p(0) / p(2), p(1) / p(2), 1.0};
//       const vec2_t pixel = (K * pt).head(2);
//       cv::Point2f cv_pixel(pixel(0), pixel(1));
//       if (i < 4) {
//         cv::circle(frame_vis, cv_pixel, 3, cv::Scalar(0, 255, 0), -1);
//       } else {
//         cv::circle(frame_vis, cv_pixel, 3, cv::Scalar(0, 0, 255), -1);
//       }
//     }
//
//     // Visualize
//     cv::imshow("Camera", frame_vis);
//     auto key = cv::waitKey(1);
//
//     // Events handler
//     if (key == 'c') {
//       frames_init.push_back(frame.clone());
//       pose_idx++;
//
//       if ((pose_idx + 1) >= poses_init.size()) {
//         return 1;
//       }
//
//     } else if (key == 'q') {
//       return -1;
//     }
//
//     return 0;
//   }
// };
//
// int test_calib_mono() {
//   // Setup
// 	calib_target_t target;
//   graph_t graph;
//   graph.param_order.clear();
//   graph.param_order.push_back("pose_t");
//   // graph.param_order.push_back("fiducial_pose_t");
//   graph.param_order.push_back("camera_params_t");
//
//   // cv::VideoCapture camera(0);
//   // if (camera.isOpened() == false) {
//   //   return -1;
//   // }
//   // sleep(2);
//
//   // Guess the camera intrinsics and distortion
//   // cv::Mat frame;
//   // camera.read(frame);
//   const auto detector = aprilgrid_detector_t();
//   // const int cam_res[2] = {frame.cols, frame.rows};
//   const int cam_res[2] = {752, 480};
//   // const double fx = pinhole_focal(cam_res[0], 90.0);
//   // const double fy = pinhole_focal(cam_res[1], 90.0);
//   const double cx = cam_res[0] / 2.0;
//   const double cy = cam_res[1] / 2.0;
//   // const double fx = 458.654;
//   // const double fy = 457.296;
//   // const double cx = 367.215;
//   // const double cy = 248.375;
//   const double fx = 450.0;
//   const double fy = 450.0;
//   // const double cx = 361.0;
//   // const double cy = 240.0;
//   const vec4_t proj_params{fx, fy, cx, cy};
//   const vec4_t dist_params{0.0, 0.0, 0.0, 0.0};
//   // const vec4_t dist_params{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
//   // const vec4_t dist_params{-0.2, 0.07, 0.0, 0.0};
//   const pinhole_radtan4_t camera_model{cam_res, proj_params, dist_params};
//
//   // // Initialize problem
//   // calib_mono_initializer_t calib_init(target, detector, camera_model);
// 	// while (true) {
// 	//   // Grab new frame
// 	// 	camera.read(frame);
//   //
//   //   // Initialize
//   //   if (calib_init.update(frame) != 0) {
//   //     break;
//   //   }
// 	// }
//
//   // int frame_index = 0;
//   // for (const auto &frame : calib_init.frames_init) {
//   //   auto save_path = "/tmp/image" + std::to_string(frame_index) + ".jpg";
//   //   printf("saving image [%d] to [%s]\n", frame_index, save_path.c_str());
//   //   cv::imwrite(save_path, frame);
//   //   frame_index++;
//   // }
//
//   std::vector<cv::Mat> frames_init;
//   // frames_init.push_back(cv::imread("/tmp/image0.jpg"));
//   // frames_init.push_back(cv::imread("/tmp/image1.jpg"));
//   // frames_init.push_back(cv::imread("/tmp/image2.jpg"));
//   // frames_init.push_back(cv::imread("/tmp/image3.jpg"));
//
//   frames_init.push_back(cv::imread("/data/euroc_mav/cam_april/mav0/cam0/data/1403709387337837056.png"));
//   frames_init.push_back(cv::imread("/data/euroc_mav/cam_april/mav0/cam0/data/1403709388887836928.png"));
//   frames_init.push_back(cv::imread("/data/euroc_mav/cam_april/mav0/cam0/data/1403709389037837056.png"));
//   frames_init.push_back(cv::imread("/data/euroc_mav/cam_april/mav0/cam0/data/1403709393787836928.png"));
//   frames_init.push_back(cv::imread("/data/euroc_mav/cam_april/mav0/cam0/data/1403709395837837056.png"));
//   frames_init.push_back(cv::imread("/data/euroc_mav/cam_april/mav0/cam0/data/1403709397387836928.png"));
//   frames_init.push_back(cv::imread("/data/euroc_mav/cam_april/mav0/cam0/data/1403709398387836928.png"));
//   frames_init.push_back(cv::imread("/data/euroc_mav/cam_april/mav0/cam0/data/1403709409187836928.png"));
//
// 	// Add fiducial pose to graph
//   const auto fiducial_id = graph_add_fiducial_pose(graph, I(4), true);
//
//   // Add camera to graph
//   const int cam_index = 0;
//   const auto cam_id = graph_add_camera(graph, cam_index, cam_res,
//                                        proj_params, dist_params);
//
//
//   // Add camera poses and BA factor to graph
//   std::vector<id_t> poses_ids;
//   std::vector<id_t> factor_ids;
//
//   size_t frame_idx = 0;
//   // for (const auto &frame : calib_init.frames_init) {
//   for (const auto &frame : frames_init) {
//     // cv::imshow("Frame", frame);
//     // cv::waitKey(0);
//
//     // Detect AprilGrid
//     aprilgrid_t grid;
//     aprilgrid_set_properties(grid,
//                              target.tag_rows,
//                              target.tag_cols,
//                              target.tag_size,
//                              target.tag_spacing);
//     const auto K = camera_model.K();
//     const auto D = camera_model.dist_params();
//     aprilgrid_detect(grid, detector, frame, K, D);
//
//     // Add camera pose
//     const mat4_t T_FC = grid.T_CF.inverse();
//     const auto cam_pose_id = graph_add_pose(graph, frame_idx, T_FC);
//     poses_ids.push_back(cam_pose_id);
//
//     // Add BA factor
//     for (const auto &tag_id : grid.ids) {
//       vec2s_t keypoints;
//       vec3s_t object_points;
//       aprilgrid_get(grid, tag_id, keypoints);
//       aprilgrid_object_points(grid, tag_id, object_points);
//
//       for (int tag_corner = 0; tag_corner < 4; tag_corner++) {
//         auto factor_id = graph_add_calib_mono_factor<pinhole_radtan4_t>(
//           graph,
//           frame_idx,
//           cam_pose_id,
//           fiducial_id,
//           cam_id,
//           tag_id,
//           tag_corner,
//           object_points[tag_corner],
//           keypoints[tag_corner],
//           10 * I(2)
//         );
//         factor_ids.push_back(factor_id);
//
//         graph.factors[factor_id]->eval();
//       }
//     }
//
//     // Increment frame index
//     frame_idx++;
//   }
//
//   // Solve graph
//
//   std::vector<double> residuals;
//   for (const auto &kv : graph.factors) {
//     const auto factor = kv.second;
//     residuals.push_back(factor->residuals.norm());
//   }
//   printf("[before] rmse reproj error: %.2f\n", rmse(residuals));
//   print_vector("[before] cam_params", graph.params[cam_id]->param);
//
//   tiny_solver_t solver;
//   solver.verbose = true;
//   solver.time_limit = 10.0;
//   solver.max_iter = 200;
//   solver.lambda = 1e-4;
//   solver.update_factor = 10;
//   solver.solve(graph);
//   printf("solver took: %fs\n", solver.solve_time);
//
//   // std::vector<double> residuals;
//   residuals.clear();
//   for (const auto &kv : graph.factors) {
//     const auto factor = kv.second;
//     residuals.push_back(factor->residuals.norm());
//   }
//   printf("[after] rmse reproj error: %.2f\n", rmse(residuals));
//   print_vector("[after] cam_params", graph.params[cam_id]->param);
//
//   // size_t marg_size = 0;
//   // size_t remain_size = 0;
//   // matx_t J = graph_jacobians(graph, &marg_size, &remain_size);
//   // mat2csv("/tmp/J.csv", J);
//
//   print_vector("fiducial", graph.params[fiducial_id]->param);
//
//   return 0;
// }

void test_suite() {
  // MU_ADD_TEST(test_calib_mono);
}

}

MU_RUN_TESTS(proto::test_suite);

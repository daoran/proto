#ifndef AVS_HPP
#define AVS_HPP

#include <map>
#include <vector>
#include <algorithm>
#include <ceres/ceres.h>

extern "C" {
#include "xyz.h"
#include "euroc.h"
}

#include <opencv2/opencv.hpp>

/******************************************************************************
 * COMPUTER-VISION
 *****************************************************************************/

///////////
// UTILS //
///////////

/**
 * Extend a std::vector with another
 */
template <typename T>
void extend_vector(std::vector<T> &destination, const std::vector<T> &source) {
  destination.insert(destination.end(), source.begin(), source.end());
}

/**
 * Returns mean
 */
real_t mean(const std::vector<real_t> &x) {
  real_t sum = 0.0;
  for (const auto i : x) {
    sum += i;
  }

  const real_t N = x.size();
  return sum / N;
}

/**
 * Returns median
 */
real_t median(const std::vector<real_t> &v) {
  // sort values
  std::vector<real_t> v_copy = v;
  std::sort(v_copy.begin(), v_copy.end());

  // obtain median
  if (v_copy.size() % 2 == 1) {
    // return middle value
    return v_copy[v_copy.size() / 2];

  } else {
    // grab middle two values and calc mean
    const real_t a = v_copy[v_copy.size() / 2];
    const real_t b = v_copy[(v_copy.size() / 2) - 1];
    return (a + b) / 2.0;
  }
}

/**
 * Returns RMSE
 */
real_t rmse(const std::vector<real_t> &residuals) {
  real_t sse = 0.0;
  for (const auto r : residuals) {
    sse += r * r;
  }

  real_t n = residuals.size();
  real_t mse = sse / n;
  return sqrt(mse);
}

/**
 * Returns variance
 */
real_t var(const std::vector<real_t> &x) {
  const double mu = mean(x);
  const double N = x.size();

  double sum = 0.0;
  for (const auto x_i : x) {
    sum += pow(x_i - mu, 2);
  }

  return sum / (N - 1.0);
}

/**
 * Convert cv::Mat type to string
 */
std::string type2str(const int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

/**
 * Convert std::vector<cv::KeyPoint> to std::vector<cv::Point2f>
 */
std::vector<cv::Point2f> kps2pts(const std::vector<cv::KeyPoint> &kps) {
  std::vector<cv::Point2f> pts;
  for (const auto &kp : kps) {
    pts.push_back(kp.pt);
  }
  return pts;
}

/**
 * Print Keypoint
 */
void print_keypoint(const cv::KeyPoint &kp) {
  printf("angle: %f\n", kp.angle);
  printf("class_id: %d\n", kp.class_id);
  printf("octave: %d\n", kp.octave);
  printf("pt: [%.2f, %.2f]\n", kp.pt.x, kp.pt.y);
  printf("response: %f\n", kp.response);
  printf("size: %f\n", kp.size);
}

/**
 * Sort Keypoints
 */
void sort_keypoints(std::vector<cv::KeyPoint> &kps) {
  std::sort(kps.begin(), kps.end(), [](cv::KeyPoint a, cv::KeyPoint b) {
    return a.response > b.response;
  });
}

/**
 * Given a set of keypoints `kps` make sure they are atleast `min_dist` pixels
 * away from each other, if they are not remove them.
 */
std::vector<cv::KeyPoint> spread_keypoints(
    const cv::Mat &image,
    const std::vector<cv::KeyPoint> &kps,
    const int min_dist = 10,
    const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>(),
    const bool debug = false) {
  // Pre-check
  std::vector<cv::KeyPoint> kps_filtered;
  if (kps.size() == 0) {
    return kps_filtered;
  }

  // Setup
  const int img_w = image.size().width;
  const int img_h = image.size().height;
  cv::Mat A = cv::Mat::zeros(img_h, img_w, CV_8UC1);

  // Loop through previous keypoints
  for (const auto kp : kps_prev) {
    // Fill the area of the matrix where the next keypoint cannot be around
    const int px = kp.pt.x;
    const int py = kp.pt.y;
    const int rs = std::max(py - min_dist, 0);
    const int re = std::min(py + min_dist + 1, img_h - 1);
    const int cs = std::max(px - min_dist, 0);
    const int ce = std::min(px + min_dist + 1, img_w - 1);

    for (size_t i = rs; i < re; i++) {
      for (size_t j = cs; j < ce; j++) {
        A.at<uint8_t>(i, j) = 255;
      }
    }
  }

  // Loop through keypoints
  for (const auto kp : kps) {
    // Check if point is ok to be added to results
    const int px = kp.pt.x;
    const int py = kp.pt.y;
    if (A.at<uint8_t>(py, px) > 0) {
      continue;
    }
    kps_filtered.push_back(kp);

    // Fill the area of the matrix where the next keypoint cannot be around
    const int rs = std::max(py - min_dist, 0);
    const int re = std::min(py + min_dist + 1, img_h - 1);
    const int cs = std::max(px - min_dist, 0);
    const int ce = std::min(px + min_dist + 1, img_w - 1);

    for (size_t i = rs; i < re; i++) {
      for (size_t j = cs; j < ce; j++) {
        A.at<uint8_t>(i, j) = 255;
      }
    }
  }

  // Debug
  if (debug) {
    cv::imshow("Debug", A);
    cv::waitKey(0);
  }

  return kps_filtered;
}

/**
 * Returns number of inliers, outliers and total from inlier vector.
 */
void inlier_stats(const std::vector<uchar> inliers,
                  size_t &num_inliers,
                  size_t &num_outliers,
                  size_t &num_total,
                  float &inlier_ratio) {
  num_inliers = 0;
  num_outliers = 0;
  num_total = inliers.size();
  for (size_t i = 0; i < inliers.size(); i++) {
    if (inliers[i]) {
      num_inliers++;
    } else {
      num_outliers++;
    }
  }

  inlier_ratio = (float) num_inliers / (float) num_total;
}

/**
 * Filter Outliers
 */
void filter_outliers(std::vector<cv::KeyPoint> &kps_i,
                     std::vector<cv::KeyPoint> &kps_j,
                     const std::vector<uchar> &inliers) {
  std::vector<cv::KeyPoint> a;
  std::vector<cv::KeyPoint> b;

  for (size_t i = 0; i < inliers.size(); i++) {
    if (inliers[i]) {
      a.push_back(kps_i[i]);
      b.push_back(kps_j[i]);
    }
  }
  kps_i = a;
  kps_j = b;
}

/**
 * Check parallax
 */
void check_parallax(const camera_params_t &cam0_params,
                    const camera_params_t &cam1_params,
                    const extrinsic_t &cam0_ext,
                    const extrinsic_t &cam1_ext,
                    const std::vector<cv::KeyPoint> &kps0,
                    const std::vector<cv::KeyPoint> &kps1,
                    const real_t parallax_threshold,
                    std::vector<uchar> &inliers) {
  assert(parallax_threshold > 0);
  assert(kps0.size() == kps1.size());

  // Form projection matrices P_i and P_j
  const real_t *params0 = cam0_params.data;
  const real_t *params1 = cam1_params.data;

  real_t I4[4 * 4] = {0};
  eye(I4, 4, 4);

  POSE2TF(cam0_ext.data, T_BC0);
  POSE2TF(cam1_ext.data, T_BC1);
  TF_INV(T_BC0, T_C0B);
  TF_CHAIN(T_C0C1, 2, T_C0B, T_BC1);
  TF_INV(T_C0C1, T_C1C0);

  real_t P0[4 * 4] = {0};
  real_t P1[4 * 4] = {0};
  pinhole_projection_matrix(params0, I4, P0);
  pinhole_projection_matrix(params1, T_C0C1, P1);

  // Check parallax
  inliers.clear();
  const size_t N = kps0.size();
  for (size_t i = 0; i < N; i++) {
    // Undistort
    const real_t z0_in[2] = {kps0[i].pt.x, kps0[i].pt.y};
    const real_t z1_in[2] = {kps1[i].pt.x, kps1[i].pt.y};
    real_t z0[2] = {0};
    real_t z1[2] = {0};
    cam0_params.undistort_func(params0, z0_in, z0);
    cam1_params.undistort_func(params1, z1_in, z1);

    // Triangulate
    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    linear_triangulation(P0, P1, z0, z1, p_C0);
    tf_point(T_C1C0, p_C0, p_C1);
    if (p_C0[2] < 0 && p_C1[2] < 0) {
      inliers.push_back(false);
      continue;
    }

    // Check parallax
    DOT(p_C0, 1, 3, p_C1, 3, 1, numerator);
    const double denominator = vec3_norm(p_C0) * vec3_norm(p_C1);
    const double parallax = rad2deg(acos(numerator[0] / denominator));
    if (parallax < parallax_threshold) {
      inliers.push_back(false);
    } else {
      inliers.push_back(true);
    }
  }
}

/******************************************************************************
 * STATE-ESTIMATION
 *****************************************************************************/

/**
 * Pose local parameterization
 */
struct PoseLocalParameterization : public ceres::LocalParameterization {
  /** Plus */
  virtual bool Plus(const double *x,
                    const double *dx,
                    double *x_plus_dx) const {
    x_plus_dx[0] = x[0] + dx[0];
    x_plus_dx[1] = x[1] + dx[1];
    x_plus_dx[2] = x[2] + dx[2];

    double dq[4] = {0};
    quat_delta(dx + 3, dq);
    quat_mul(x + 3, dq, x_plus_dx + 3);
    quat_normalize(x_plus_dx + 3);

    return true;
  }

  /** Compute Jacobian */
  virtual bool ComputeJacobian(const double *x, double *J) const {
    // clang-format off
    J[0]  = 1; J[1]  = 0; J[2]  = 0;  J[3] = 0; J[4]  = 0; J[5]  = 0;
    J[6]  = 0; J[7]  = 1; J[8]  = 0;  J[9] = 0; J[10] = 0; J[11] = 0;
    J[12] = 0; J[13] = 0; J[14] = 1; J[15] = 0; J[16] = 0; J[17] = 0;
    J[18] = 0; J[19] = 0; J[20] = 0; J[21] = 1; J[22] = 0; J[23] = 0;
    J[24] = 0; J[25] = 0; J[26] = 0; J[27] = 0; J[28] = 1; J[29] = 0;
    J[30] = 0; J[31] = 0; J[32] = 0; J[33] = 0; J[34] = 0; J[35] = 1;
    J[36] = 0; J[37] = 0; J[38] = 0; J[39] = 0; J[40] = 0; J[41] = 0;
    // clang-format on
    return true;
  }

  /** Return global size */
  virtual int GlobalSize() const { return 7; }

  /** Return local size */
  virtual int LocalSize() const { return 6; }

  /** Form delta quaternion `dq` from a small rotation vector `dalpha`. */
  static void quat_delta(const double dalpha[3], double dq[4]) {
    assert(dalpha != NULL);
    assert(dq != NULL);

    const double half_norm = 0.5 * vec_norm(dalpha, 3);
    const double k = sinc(half_norm) * 0.5;
    const double vector[3] = {k * dalpha[0], k * dalpha[1], k * dalpha[2]};
    double scalar = cos(half_norm);

    dq[0] = scalar;
    dq[1] = vector[0];
    dq[2] = vector[1];
    dq[3] = vector[2];
  }

  /** Quaternion left-multiply `p` with `q`, results are outputted to `r`. */
  static void quat_lmul(const double p[4], const double q[4], double r[4]) {
    assert(p != NULL);
    assert(q != NULL);
    assert(r != NULL);

    const double pw = p[0];
    const double px = p[1];
    const double py = p[2];
    const double pz = p[3];

    r[0] = pw * q[0] - px * q[1] - py * q[2] - pz * q[3];
    r[1] = px * q[0] + pw * q[1] - pz * q[2] + py * q[3];
    r[2] = py * q[0] + pz * q[1] + pw * q[2] - px * q[3];
    r[3] = pz * q[0] - py * q[1] + px * q[2] + pw * q[3];
  }

  /** Quaternion multiply `p` with `q`, results are outputted to `r`. */
  static void quat_mul(const double p[4], const double q[4], double r[4]) {
    assert(p != NULL);
    assert(q != NULL);
    assert(r != NULL);
    quat_lmul(p, q, r);
  }

  /** Return Quaternion norm */
  static double quat_norm(const double q[4]) {
    return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  }

  /** Normalize Quaternion */
  static void quat_normalize(double q[4]) {
    const double n = quat_norm(q);
    q[0] = q[0] / n;
    q[1] = q[1] / n;
    q[2] = q[2] / n;
    q[3] = q[3] / n;
  }
};

/** Estimate Relative Pose **/
real_t estimate_pose(std::map<int, camera_params_t> cam_ints,
                     std::map<int, extrinsic_t> cam_exts,
                     std::map<size_t, std::shared_ptr<Feature>> &features,
                     pose_t &pose_km1,
                     pose_t &pose_k,
                     const int max_iter = 10,
                     const int max_num_threads = 2,
                     const bool verbose = true,
                     const bool fix_features = true,
                     const bool fix_camera_intrinsics = true,
                     const bool fix_camera_extrinsics = true) {
  // Setup
  ceres::Problem::Options prob_options;
  ceres::Problem problem{prob_options};
  PoseLocalParameterization *pose_param = new PoseLocalParameterization();
  ceres::LossFunction *loss_fn = new ceres::CauchyLoss(1.0);

  // Add camera intrinsics and extrinsics to problem
  for (auto &[cam_id, cam_int] : cam_ints) {
    problem.AddParameterBlock(cam_int.data, 8);
    if (fix_camera_intrinsics) {
      problem.SetParameterBlockConstant(cam_int.data);
    }
  }
  for (auto &[cam_id, cam_ext] : cam_exts) {
    problem.AddParameterBlock(cam_ext.data, 7);
    problem.SetParameterization(cam_ext.data, pose_param);
    if (fix_camera_extrinsics) {
      problem.SetParameterBlockConstant(cam_ext.data);
    }
  }

  // Add poses at km1 and k, fix pose at km1
  problem.AddParameterBlock(pose_km1.data, 7);
  problem.AddParameterBlock(pose_k.data, 7);
  problem.SetParameterization(pose_km1.data, pose_param);
  problem.SetParameterization(pose_k.data, pose_param);
  problem.SetParameterBlockConstant(pose_km1.data);

  // Add vision factors
  std::vector<VisionFactor *> factors;
  for (auto &[fid, feature] : features) {
    // Pre-check
    if (feature->initialized == false) {
      continue;
    }

    // Add feature to problem
    problem.AddParameterBlock(feature->data, 3);
    if (fix_features) {
      problem.SetParameterBlockConstant(feature->data);
    }

    // Add camera factors at pose_k
    for (const auto &[cam_idx, kp] : feature->keypoints[pose_k.ts]) {
      // Form factor
      const real_t z[2] = {kp.pt.x, kp.pt.y};
      auto res_fn = new VisionFactor{pose_k.ts, cam_idx, fid, z};
      factors.push_back(res_fn);

      // Add factor to problem
      std::vector<double *> param_blocks;
      param_blocks.push_back(pose_k.data);
      param_blocks.push_back(cam_exts.at(cam_idx).data);
      param_blocks.push_back(feature->data);
      param_blocks.push_back(cam_ints.at(cam_idx).data);
      problem.AddResidualBlock(res_fn, loss_fn, param_blocks);
    }

    // Add camera factors at pose_km1
    for (const auto &[cam_idx, kp] : feature->keypoints[pose_km1.ts]) {
      // Form factor
      const real_t z[2] = {kp.pt.x, kp.pt.y};
      auto res_fn = new VisionFactor{pose_km1.ts, cam_idx, fid, z};
      factors.push_back(res_fn);

      // Add factor to problem
      std::vector<double *> param_blocks;
      param_blocks.push_back(pose_km1.data);
      param_blocks.push_back(cam_exts.at(cam_idx).data);
      param_blocks.push_back(feature->data);
      param_blocks.push_back(cam_ints.at(cam_idx).data);
      problem.AddResidualBlock(res_fn, loss_fn, param_blocks);
    }
  }

  // Evaluate reprojection errors
  std::vector<double> reproj_errors;
  std::map<size_t, std::vector<double>> feature_errors;
  auto eval_residuals = [&]() {
    for (auto &factor : factors) {
      double r[2] = {0.0, 0.0};

      std::vector<double *> params;
      if (factor->ts == pose_km1.ts) {
        params.push_back(pose_km1.data);
      } else {
        params.push_back(pose_k.data);
      }
      params.push_back(cam_exts.at(factor->cam_id).data);
      params.push_back(features[factor->feature_id]->data);
      params.push_back(cam_ints.at(factor->cam_id).data);
      factor->Evaluate(params.data(), r, nullptr);

      const auto reproj_error = sqrt(r[0] * r[0] + r[1] * r[1]);
      reproj_errors.push_back(reproj_error);
      feature_errors[factor->feature_id].push_back(reproj_error);
    }
  };

  // Solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = max_iter;
  options.num_threads = max_num_threads;
  ceres::Solver::Summary summary;

  // eval_residuals();
  // printf("Before\n");
  // printf("-----------------------\n");
  // printf("mean reproj error: %f\n", mean(reproj_errors));
  // printf("median reproj error: %f\n", median(reproj_errors));
  // printf("rmse reproj error: %f\n", rmse(reproj_errors));
  // printf("var reproj error: %f\n", var(reproj_errors));

  ceres::Solve(options, &problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl << std::endl;
  }

  eval_residuals();
  if (reproj_errors.size() < 10) {
    return std::numeric_limits<double>::max();
  }

  // printf("Estimate pose\n");
  // printf("-----------------------\n");
  // printf("mean reproj error: %f\n", mean(reproj_errors));
  // printf("median reproj error: %f\n", median(reproj_errors));
  // printf("rmse reproj error: %f\n", rmse(reproj_errors));
  // printf("var reproj error: %f\n", var(reproj_errors));

  const auto threshold = 3.0 * std::sqrt(var(reproj_errors));
  std::vector<size_t> outliers;
  for (auto &[fid, feature] : features) {
    if (mean(feature_errors[fid]) >= 5.0) {
      outliers.push_back(fid);
    }
  }
  // printf("removed %ld outliers out of %ld\n", outliers.size(), features.size());
  for (const auto fid : outliers) {
    features.erase(fid);
  }

  return rmse(reproj_errors);
}

struct FrameSet {
  timestamp_t ts;
  std::vector<size_t> feature_ids;
  pose_t pose;
};

/** Two State Filter **/
struct TSF {
  // Fronend
  GridDetector detector;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();

  // Settings
  int max_keypoints = 500;
  bool enable_clahe = true;
  int parallax_threshold = 1.0;
  int max_length = 40;
  int min_length = 5;

  // Calibrations
  std::map<int, camera_params_t> cam_ints;
  std::map<int, extrinsic_t> cam_exts;

  // Features
  size_t next_feature_id = 0;
  std::map<size_t, std::shared_ptr<Feature>> features;
  std::map<size_t, std::shared_ptr<Feature>> old_features;

  // Data
  bool initialized = false;
  timestamp_t prev_ts = -1;
  cv::Mat prev_frame0;
  cv::Mat prev_frame1;
  size_t frame_index = 0;
  std::vector<pose_t> pose_hist;

  // Poses
  pose_t pose_init;
  pose_t pose_km1;
  pose_t pose_k;

  /** Constructor **/
  TSF(std::map<int, camera_params_t> &cam_ints_,
      std::map<int, extrinsic_t> &cam_exts_)
      : cam_ints{cam_ints_}, cam_exts{cam_exts_} {
    real_t pose0[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    pose_setup(&pose_init, 0, pose0);
    pose_setup(&pose_km1, 0, pose0);
    pose_setup(&pose_k, 0, pose0);
  }

  /** Destructor **/
  virtual ~TSF() = default;

  /** Set initial pose **/
  void set_initial_pose(const real_t T_WB[4 * 4]) {
    TF_VECTOR(T_WB, pose_vec);
    pose_setup(&pose_init, 0, pose_vec);
  }

  /** Add Feature **/
  void _add_feature(const timestamp_t ts,
                    const cv::KeyPoint &kp0,
                    const cv::KeyPoint &kp1) {
    const auto fid = next_feature_id;
    features[fid] = std::make_shared<Feature>(fid, cam_ints, cam_exts);
    features[fid]->update(ts, 0, kp0);
    features[fid]->update(ts, 1, kp1);
    next_feature_id += 1;
  }

  /** Update Feature **/
  void _update_feature(const size_t fid,
                       const timestamp_t ts,
                       const cv::KeyPoint &kp0,
                       const cv::KeyPoint &kp1) {
    features[fid]->update(ts, 0, kp0);
    features[fid]->update(ts, 1, kp1);

    if (features[fid]->length() >= features[fid]->max_length) {
      _remove_feature(fid);
    }
  }

  /** Remove Feature **/
  void _remove_feature(const size_t fid) {
    old_features[fid] = features[fid];
    features.erase(fid);
  }

  /** Get keypoints **/
  void _get_keypoints(std::vector<size_t> &feature_ids,
                      std::vector<cv::KeyPoint> &kps0,
                      std::vector<cv::KeyPoint> &kps1) {
    for (const auto &[fid, feature] : features) {
      auto keypoints = feature->get_keypoints();
      feature_ids.push_back(fid);
      kps0.push_back(keypoints[0]);
      kps1.push_back(keypoints[1]);
    }
  }

  void _initialize_features(const timestamp_t ts) {
    const int max_iter = 2;
    const int max_num_threads = 4;
    const bool verbose = false;
    const bool fix_features = false;
    const bool fix_camera_intrinsics = true;
    const bool fix_camera_extrinsics = true;

    // Initialize features
    POSE2TF(pose_k.data, T_WB);
    std::set<size_t> new_fids;
    for (auto &[fid, feature] : features) {
      if (feature->initialized) {
        continue;
      }
      if (feature->initialize(ts, T_WB) == true) {
        new_fids.insert(feature->feature_id);
      }
    }

    // Refine initialized features
    // -- Setup
    ceres::Problem::Options prob_options;
    ceres::Problem problem{prob_options};
    PoseLocalParameterization *pose_param = new PoseLocalParameterization();
    ceres::LossFunction *loss_fn = nullptr;

    // -- Add camera intrinsics and extrinsics to problem
    for (auto &[cam_id, cam_int] : cam_ints) {
      problem.AddParameterBlock(cam_int.data, 8);
      if (fix_camera_intrinsics) {
        problem.SetParameterBlockConstant(cam_int.data);
      }
    }
    for (auto &[cam_id, cam_ext] : cam_exts) {
      problem.AddParameterBlock(cam_ext.data, 7);
      problem.SetParameterization(cam_ext.data, pose_param);
      if (fix_camera_extrinsics) {
        problem.SetParameterBlockConstant(cam_ext.data);
      }
    }

    // -- Add poses at km1 and k, fix pose at km1
    problem.AddParameterBlock(pose_k.data, 7);
    problem.SetParameterization(pose_k.data, pose_param);
    problem.SetParameterBlockConstant(pose_k.data);

    // -- Add vision factors
    std::map<size_t, VisionFactor *> factors;
    for (auto &[fid, feature] : features) {
      // Pre-check
      if (feature->initialized == false) {
        continue;
      }

      // Add feature to problem
      problem.AddParameterBlock(feature->data, 3);

      // Add camera factors at pose_k
      for (const auto &[cam_idx, kp] : feature->keypoints[pose_k.ts]) {
        // Form factor
        const real_t z[2] = {kp.pt.x, kp.pt.y};
        auto res_fn = new VisionFactor{pose_k.ts, cam_idx, fid, z};
        factors[fid] = res_fn;

        // Add factor to problem
        std::vector<double *> param_blocks;
        param_blocks.push_back(pose_k.data);
        param_blocks.push_back(cam_exts.at(cam_idx).data);
        param_blocks.push_back(feature->data);
        param_blocks.push_back(cam_ints.at(cam_idx).data);
        problem.AddResidualBlock(res_fn, loss_fn, param_blocks);
      }
    }

    // Solve
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = verbose;
    options.max_num_iterations = max_iter;
    options.num_threads = max_num_threads;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (verbose) {
      std::cout << summary.FullReport() << std::endl << std::endl;
    }

    // Evaluate reprojection errors
    // std::vector<double> reproj_errors;
    // std::map<size_t, double> feature_errors;
    // auto eval_residuals = [&]() {
    //   for (auto &[fid, factor] : factors) {
    //     double r[2] = {0.0, 0.0};

    //     std::vector<double *> params;
    //     params.push_back(pose_k.data);
    //     params.push_back(cam_exts.at(factor->cam_id).data);
    //     params.push_back(features[factor->feature_id]->data);
    //     params.push_back(cam_ints.at(factor->cam_id).data);
    //     factor->Evaluate(params.data(), r, nullptr);

    //     const auto reproj_error = sqrt(r[0] * r[0] + r[1] * r[1]);
    //     reproj_errors.push_back(reproj_error);
    //     feature_errors[factor->feature_id] = reproj_error;
    //   }
    // };
    // eval_residuals();

    // printf("\n");
    // printf("Initialize Features:\n");
    // printf("--------------------\n");
    // printf("mean reproj error: %f\n", mean(reproj_errors));
    // printf("median reproj error: %f\n", median(reproj_errors));
    // printf("rmse reproj error: %f\n", rmse(reproj_errors));
    // printf("var reproj error: %f\n", var(reproj_errors));
    // printf("\n");

    // const auto threshold = 3.0 * std::sqrt(var(reproj_errors));
    // std::vector<size_t> outliers;
    // for (auto &[fid, feature] : features) {
    //   if (feature_errors[fid] >= threshold) {
    //     outliers.push_back(fid);
    //   }
    // }
    // for (const auto fid : outliers) {
    //   features.erase(fid);
    // }
  }

  void _initialize(const timestamp_t ts) {
    // Pre-checks
    if (initialized) {
      return;
    } else if (frame_index < min_length) {
      return;
    }

    // Initialize features
    pose_setup(&pose_k, ts, pose_init.data);
    pose_setup(&pose_km1, ts, pose_init.data);
    _initialize_features(ts);
    initialized = true;
  }

  /** Detect Features **/
  void detect(const timestamp_t ts,
              const cv::Mat &frame0,
              const cv::Mat &frame1) {
    // Get previous keypoints from cam0
    std::vector<size_t> feature_ids;
    std::vector<cv::KeyPoint> kps0;
    std::vector<cv::KeyPoint> kps1;
    _get_keypoints(feature_ids, kps0, kps1);

    // Detect new
    std::vector<cv::KeyPoint> kps0_new;
    std::vector<cv::KeyPoint> kps1_new;
    detector.detect(frame0, kps0_new, kps0);
    if (kps0_new.size() < 10) {
      return;
    }

    // Track in space
    std::vector<uchar> optflow_inliers;
    optflow_track(frame0, frame1, kps0_new, kps1_new, optflow_inliers);
    filter_outliers(kps0_new, kps1_new, optflow_inliers);

    // Ransac in space
    std::vector<uchar> ransac_inliers;
    ransac(kps0_new,
           kps1_new,
           cam_ints[0].undistort_func,
           cam_ints[1].undistort_func,
           cam_ints[0].data,
           cam_ints[1].data,
           ransac_inliers);
    filter_outliers(kps0_new, kps1_new, ransac_inliers);

    // Check parallax
    std::vector<uchar> parallax_inliers;
    const real_t parallax_threshold = 1.0;
    check_parallax(cam_ints[0],
                   cam_ints[1],
                   cam_exts[0],
                   cam_exts[1],
                   kps0_new,
                   kps1_new,
                   parallax_threshold,
                   parallax_inliers);
    filter_outliers(kps0_new, kps1_new, parallax_inliers);

    // Add new features
    for (size_t i = 0; i < kps0_new.size(); i++) {
      _add_feature(ts, kps0_new[i], kps1_new[i]);
    }
  }

  /** Track features **/
  bool track(const timestamp_t ts,
             const cv::Mat &frame0,
             const cv::Mat &frame1) {
    // Pre-check
    if (prev_frame0.empty() || prev_frame1.empty()) {
      return false;
    }

    // Get previous keypoints from cam0
    std::vector<size_t> feature_ids;
    std::vector<cv::KeyPoint> kps0_km1;
    std::vector<cv::KeyPoint> kps1_km1;
    _get_keypoints(feature_ids, kps0_km1, kps1_km1);

    // Track in time and space
    std::vector<uchar> in0;
    std::vector<uchar> in1;
    std::vector<cv::KeyPoint> kps0_k;
    std::vector<cv::KeyPoint> kps1_k;
    optflow_track(prev_frame0, frame0, kps0_km1, kps0_k, in0);
    optflow_track(prev_frame1, frame1, kps1_km1, kps1_k, in1);

    // Ransac in time
    std::vector<uchar> rs0;
    std::vector<uchar> rs1;
    auto cam0_undistort_func = cam_ints[0].undistort_func;
    auto cam1_undistort_func = cam_ints[1].undistort_func;
    auto cam0_params = cam_ints[0].data;
    auto cam1_params = cam_ints[1].data;
    ransac(kps0_km1, kps0_k, cam0_undistort_func, cam0_params, rs0);
    ransac(kps1_km1, kps1_k, cam1_undistort_func, cam1_params, rs1);

    // Update inliers and remove outliers
    for (size_t i = 0; i < in0.size(); i++) {
      const auto fid = feature_ids[i];
      if (in0[i] && in1[i] && rs0[i] && rs1[i]) {
        _update_feature(fid, ts, kps0_k[i], kps1_k[i]);
      } else {
        _remove_feature(fid);
      }
    }

    // Initialize or track
    if (initialized == false && frame_index >= min_length) {
      _initialize(ts);
      return true;

    } else if (initialized) {
      // Step 1. Estimate current pose
      pose_setup(&pose_k, ts, pose_km1.data);
      const int max_iter = 2;
      const int max_num_threads = 4;
      const bool verbose = false;
      const auto rmse = estimate_pose(cam_ints,
                                      cam_exts,
                                      features,
                                      pose_km1,
                                      pose_k,
                                      max_iter,
                                      max_num_threads,
                                      verbose);

      // Step 2. Initialize new features with current pose estimate
      _initialize_features(ts);

      // Step 3. Refine poses and features currently tracking
      estimate_pose(cam_ints,
                    cam_exts,
                    features,
                    pose_km1,
                    pose_k,
                    max_iter,
                    max_num_threads,
                    verbose,
                    false);
    }

    return true;
  }

  /** Visualize **/
  void _visualize(const cv::Mat &frame0, const cv::Mat &frame1) {
    // Get previous keypoints from cam0
    std::vector<size_t> feature_ids;
    std::vector<cv::KeyPoint> kps0_k;
    std::vector<cv::KeyPoint> kps1_k;
    _get_keypoints(feature_ids, kps0_k, kps1_k);

    // Visualize
    const cv::Scalar red{0, 0, 255};
    cv::Mat viz;
    cv::cvtColor(frame0, viz, cv::COLOR_GRAY2RGB);
    cv::drawKeypoints(viz, kps0_k, viz, red);
    cv::imshow("viz", viz);
  }

  /** Update **/
  void update(const timestamp_t ts,
              const cv::Mat &frame0,
              const cv::Mat &frame1) {
    // Apply CLAHE
    if (enable_clahe) {
      clahe->apply(frame0, frame0);
      clahe->apply(frame1, frame1);
    }

    // Detect and track
    detect(ts, frame0, frame1);
    track(ts, frame0, frame1);

    // Visualize
    // _visualize(frame0, frame1);
    if (initialized) {
      pose_hist.emplace_back(pose_k);
    }

    // Update
    pose_setup(&pose_km1, ts, pose_k.data);
    prev_ts = ts;
    prev_frame0 = frame0.clone();
    prev_frame1 = frame1.clone();
    frame_index += 1;
  }
};

#endif // AVS_HPP

//////////////////////////////////////////////////////////////////////////////
//                              UNIT-TESTS                                  //
//////////////////////////////////////////////////////////////////////////////

#ifdef AVS_UNITTESTS

// UNITESTS GLOBAL VARIABLES
static int nb_tests = 0;
static int nb_passed = 0;
static int nb_failed = 0;

#define ENABLE_TERM_COLORS 0
#if ENABLE_TERM_COLORS == 1
#define TERM_RED "\x1B[1;31m"
#define TERM_GRN "\x1B[1;32m"
#define TERM_WHT "\x1B[1;37m"
#define TERM_NRM "\x1B[1;0m"
#else
#define TERM_RED
#define TERM_GRN
#define TERM_WHT
#define TERM_NRM
#endif

/**
 * Run unittests
 * @param[in] test_name Test name
 * @param[in] test_ptr Pointer to unittest
 */
void run_test(const char *test_name, int (*test_ptr)()) {
  if ((*test_ptr)() == 0) {
    printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
    fflush(stdout);
    nb_passed++;
  } else {
    printf(TERM_RED "FAILED!\n" TERM_NRM);
    fflush(stdout);
    nb_failed++;
  }
  nb_tests++;
}

/**
 * Add unittest
 * @param[in] TEST Test function
 */
#define TEST(TEST_FN) run_test(#TEST_FN, TEST_FN);

/**
 * Unit-test assert
 * @param[in] TEST Test condition
 */
#define TEST_ASSERT(TEST)                                                      \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      printf(TERM_RED "ERROR!" TERM_NRM " [%s:%d] %s FAILED!\n",               \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST);                                                           \
      return -1;                                                               \
    }                                                                          \
  } while (0)

int test_feature_grid() {
  int image_width = 640;
  int image_height = 480;
  int grid_rows = 3;
  int grid_cols = 4;
  FeatureGrid grid{image_width, image_height, grid_rows, grid_cols};

  const float dx = (image_width / grid_cols);
  const float dy = (image_height / grid_rows);
  for (int i = 0; i < grid_rows; i++) {
    for (int j = 0; j < grid_cols; j++) {
      const int pixel_x = dx * j + dx / 2;
      const int pixel_y = dy * i + dy / 2;
      grid.add(pixel_x, pixel_y);
    }
  }
  grid.debug(true);

  return 0;
}

int test_spread_keypoints() {
  // Load image
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.png";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  // Detect keypoints
  const cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  std::vector<cv::KeyPoint> kps;
  detector->detect(img, kps);

  // Spread keypoints
  const int min_dist = 5;
  const std::vector<cv::KeyPoint> kps_prev;
  const bool debug = false;
  const auto kps_new = spread_keypoints(img, kps, min_dist, kps_prev, debug);

  // Assert keypoints are a Euclidean distance away from each other
  for (const auto kp : kps_new) {
    for (const auto kp_ref : kps_new) {
      const int px = kp.pt.x;
      const int py = kp.pt.y;
      const int px_ref = kp_ref.pt.x;
      const int py_ref = kp_ref.pt.y;

      const int dx = std::abs(px_ref - px);
      const int dy = std::abs(py_ref - py);
      if (dx == 0 && dy == 0) {
        continue;
      }

      const int dist = sqrt(dx * dx + dy * dy);
      TEST_ASSERT(dist >= min_dist);
    }
  }

  return 0;
}

int test_grid_detect() {
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  std::vector<cv::KeyPoint> kps_prev;
  std::vector<cv::KeyPoint> kps_new;
  GridDetector detector;
  detector.detect(img, kps_new, kps_prev, true);
  cv::waitKey(0);

  return 0;
}

int test_optflow_track() {
  const auto img_i_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img_j_path = "./test_data/frontend/cam1/1403715297312143104.jpg";
  const auto img_i = cv::imread(img_i_path, cv::IMREAD_GRAYSCALE);
  const auto img_j = cv::imread(img_j_path, cv::IMREAD_GRAYSCALE);

  // Detect keypoints
  std::vector<cv::KeyPoint> kps_new;
  GridDetector det;
  det.detect(img_i, kps_new);

  // Optical-flow track
  std::vector<cv::KeyPoint> kps_i = kps_new;
  std::vector<cv::KeyPoint> kps_j = kps_new;
  std::vector<uchar> inliers;
  optflow_track(img_i, img_j, kps_i, kps_j, inliers);

  return 0;
}

int test_reproj_filter() {
  const auto img_i_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img_j_path = "./test_data/frontend/cam1/1403715297312143104.jpg";
  auto img_i = cv::imread(img_i_path, cv::IMREAD_GRAYSCALE);
  auto img_j = cv::imread(img_j_path, cv::IMREAD_GRAYSCALE);

  // Detect keypoints
  std::vector<cv::KeyPoint> kps_new;
  GridDetector det;
  det.detect(img_i, kps_new);

  // Optical-flow track
  std::vector<cv::KeyPoint> kps_i = kps_new;
  std::vector<cv::KeyPoint> kps_j = kps_new;
  std::vector<uchar> optflow_inliers;
  optflow_track(img_i, img_j, kps_i, kps_j, optflow_inliers);

  // clang-format off
  const int cam_res[2] = {752, 480};
  real_t cam0_int[8] = {
    458.654, 457.296, 367.215, 248.375, -0.28340811,
    0.07395907, 0.00019359, 1.76187114e-05
  };
  real_t cam1_int[8] = {
    457.587, 456.134, 379.999, 255.238,
    -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05
  };
  const real_t T_SC0[4 * 4] = {
    0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
    0.0, 0.0, 0.0, 1.0
  };
  const real_t T_SC1[4 * 4] = {
    0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
    0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
    -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
    0.0, 0.0, 0.0, 1.0
  };
  TF_INV(T_SC0, T_C0S);
  TF_IDENTITY(T_C0C0);
  TF_CHAIN(T_C0C1, 2, T_C0S, T_SC1);
  // clang-format on

  TIC(reproj_filter_time);
  std::vector<cv::Point3d> points;
  std::vector<bool> reproj_inliers;
  reproj_filter(pinhole_radtan4_project,
                cam_res,
                cam0_int,
                cam1_int,
                T_C0C0,
                T_C0C1,
                kps_i,
                kps_j,
                points,
                reproj_inliers);
  PRINT_TOC("Reproj Filter", reproj_filter_time);

  // Filter keypoints
  std::map<int, std::vector<cv::KeyPoint>> keypoints;
  keypoints[0] = std::vector<cv::KeyPoint>();
  keypoints[1] = std::vector<cv::KeyPoint>();
  for (size_t n = 0; n < kps_i.size(); n++) {
    if (optflow_inliers[n] && reproj_inliers[n]) {
      keypoints[0].push_back(kps_i[n]);
      keypoints[1].push_back(kps_j[n]);
    }
  }

  // Visualize
  const auto red = cv::Scalar{0, 0, 255};
  cv::Mat img0_viz;
  cv::Mat img1_viz;
  cv::Mat viz;
  cv::cvtColor(img_i, img0_viz, cv::COLOR_GRAY2RGB);
  cv::cvtColor(img_j, img1_viz, cv::COLOR_GRAY2RGB);
  cv::drawKeypoints(img0_viz, keypoints[0], img0_viz, red);
  cv::drawKeypoints(img1_viz, keypoints[1], img1_viz, red);
  cv::hconcat(img0_viz, img1_viz, viz);
  cv::imshow("viz", viz);
  cv::waitKey(0);

  return 0;
}

class EuRoCParams {
public:
  camera_params_t cam0_params;
  camera_params_t cam1_params;
  extrinsic_t cam0_ext;
  extrinsic_t cam1_ext;

  EuRoCParams() {
    // clang-format off
    const int cam_res[2] = {752, 480};
    const char *proj_model = "pinhole";
    const char *dist_model = "radtan4";
    real_t cam0_data[8] = {
      458.654, 457.296, 367.215, 248.375,
      -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05
    };
    real_t cam1_data[8] = {
      457.587, 456.134, 379.999, 255.238,
      -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05
    };
    real_t cam0_ext_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    real_t cam1_ext_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    const real_t T_SC0[4 * 4] = {
      0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
      0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
      -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
      0.0, 0.0, 0.0, 1.0
    };
    const real_t T_SC1[4 * 4] = {
      0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
      0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
      -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
      0.0, 0.0, 0.0, 1.0
    };
    tf_vector(T_SC0, cam0_ext_data);
    tf_vector(T_SC1, cam1_ext_data);

    camera_params_setup(&cam0_params, 0, cam_res, proj_model, dist_model, cam0_data);
    camera_params_setup(&cam1_params, 1, cam_res, proj_model, dist_model, cam1_data);
    extrinsic_setup(&cam0_ext, cam0_ext_data);
    extrinsic_setup(&cam1_ext, cam1_ext_data);
    // clang-format on
  }
};

int test_tracking() {
  // Setup
  const char *data_path = "/data/euroc/V1_01";
  euroc_data_t *data = euroc_data_load(data_path);
  euroc_timeline_t *timeline = data->timeline;

  const cv::Scalar red{0, 0, 255};
  EuRoCParams euroc;
  GridDetector detector;
  cv::Mat frame0_km1;
  std::vector<cv::KeyPoint> kps0_km1;

  int imshow_wait = 1;
  for (size_t k = 0; k < timeline->num_timestamps; k++) {
    const timestamp_t ts = timeline->timestamps[k];
    const euroc_event_t *event = &timeline->events[k];

    if (event->has_cam0 && event->has_cam1) {
      struct timespec t_start = tic();
      const auto frame0_k = cv::imread(event->cam0_image, cv::IMREAD_GRAYSCALE);
      assert(frame0_k.empty() == false);

      // Track
      std::vector<cv::KeyPoint> kps0_k;
      if (k > 0) {
        std::vector<uchar> inliers;
        optflow_track(frame0_km1, frame0_k, kps0_km1, kps0_k, inliers);
        filter_outliers(kps0_km1, kps0_k, inliers);

        ransac(kps0_km1,
               kps0_k,
               euroc.cam0_params.undistort_func,
               euroc.cam0_params.undistort_func,
               euroc.cam0_params.data,
               euroc.cam0_params.data,
               inliers);
        filter_outliers(kps0_km1, kps0_k, inliers);
      }

      // Detect
      std::vector<cv::KeyPoint> kps_new;
      detector.detect(frame0_k, kps_new, kps0_km1);
      if (kps_new.size()) {
        extend_vector(kps0_k, kps_new);
      }

      // Visualize
      cv::Mat viz;
      cv::cvtColor(frame0_k, viz, cv::COLOR_GRAY2RGB);
      cv::drawKeypoints(viz, kps0_k, viz, red);
      cv::imshow("viz", viz);

      // Update
      frame0_km1 = frame0_k.clone();
      kps0_km1 = kps0_k;
      printf("track elasped: %f [s]\n", toc(&t_start));

      char key = cv::waitKey(imshow_wait);
      if (key == 'q') {
        k = timeline->num_timestamps;
      } else if (key == 's') {
        imshow_wait = 0;
      } else if (key == ' ' && imshow_wait == 1) {
        imshow_wait = 0;
      } else if (key == ' ' && imshow_wait == 0) {
        imshow_wait = 1;
      }
    }
  }

  // Clean up
  euroc_data_free(data);

  return 0;
}

int test_tsif() {
  // Setup
  const char *data_path = "/data/euroc/V1_01";
  euroc_data_t *data = euroc_data_load(data_path);
  euroc_timeline_t *timeline = data->timeline;
  EuRoCParams euroc;

  std::map<int, camera_params_t> cam_ints;
  std::map<int, extrinsic_t> cam_exts;
  cam_ints[0] = euroc.cam0_params;
  cam_ints[1] = euroc.cam1_params;
  cam_exts[0] = euroc.cam0_ext;
  cam_exts[1] = euroc.cam1_ext;
  TSF tsif{cam_ints, cam_exts};

  // const int start_index = 2500;
  const int start_index = 0;
  const timestamp_t start_ts = data->ground_truth->timestamps[start_index];
  const real_t *r_WB = data->ground_truth->p_RS_R[start_index];
  const real_t *q_WB = data->ground_truth->q_RS[start_index];
  TF_QR(q_WB, r_WB, T_WB);
  tsif.set_initial_pose(T_WB);

  FILE *gnuplot = gnuplot_init();
  gnuplot_send(gnuplot, "set title 'Plot XY'");

  auto plot = [&](const timestamp_t ts) {
    if (tsif.initialized == false) {
      return;
    }

    // Plot estimate
    // clang-format off
    std::vector<double> xvals;
    std::vector<double> yvals;
    int n = tsif.pose_hist.size();
    for (int i = 0; i < n; i++) {
      xvals.push_back(tsif.pose_hist[i].data[0]);
      yvals.push_back(tsif.pose_hist[i].data[1]);
    }
    gnuplot_send_xy(gnuplot, "$DATA1", xvals.data(), yvals.data(), n);
    fflush(gnuplot);
    // clang-format on

    // Plot ground-truth
    // clang-format off
    std::vector<double> xvals_gnd;
    std::vector<double> yvals_gnd;
    for (int i = 0; i < data->ground_truth->num_timestamps; i++) {
      if (data->ground_truth->timestamps[i] > ts) {
        break;
      }

      const real_t *r_WB = data->ground_truth->p_RS_R[i];
      xvals_gnd.push_back(r_WB[0]);
      yvals_gnd.push_back(r_WB[1]);
    }
    gnuplot_send_xy(gnuplot, "$DATA2", xvals_gnd.data(), yvals_gnd.data(), xvals_gnd.size());
    gnuplot_send(gnuplot, "plot $DATA1 with lines title 'Est', $DATA2 with lines title 'Gnd'");
    fflush(gnuplot);
    // clang-format on
  };

  int imshow_wait = 1;
  for (size_t k = 0; k < timeline->num_timestamps; k++) {
    const timestamp_t ts = timeline->timestamps[k];
    const euroc_event_t *event = &timeline->events[k];
    if (ts < start_ts) {
      continue;
    }

    if (event->has_cam0 && event->has_cam1) {
      const cv::Mat img0 = cv::imread(event->cam0_image, cv::IMREAD_GRAYSCALE);
      const cv::Mat img1 = cv::imread(event->cam1_image, cv::IMREAD_GRAYSCALE);
      assert(img0.empty() == false);
      assert(img1.empty() == false);

      printf("\n");
      printf("\n");
      printf("----------------------------------------\n");
      struct timespec t_start = tic();
      tsif.update(ts, img0, img1);
      printf("track elasped: %f [s]\n", toc(&t_start));
      printf("----------------------------------------\n");
      printf("\n");
      printf("\n");
      plot(ts);

      char key = cv::waitKey(imshow_wait);
      if (key == 'q') {
        k = timeline->num_timestamps;
      } else if (key == 's') {
        imshow_wait = 0;
      } else if (key == ' ' && imshow_wait == 1) {
        imshow_wait = 0;
      } else if (key == ' ' && imshow_wait == 0) {
        imshow_wait = 1;
      }
    }
  }

  // Clean up
  euroc_data_free(data);

  return 0;
}

void run_unittests() {
  // TEST(test_feature_grid);
  // TEST(test_spread_keypoints);
  // TEST(test_grid_detect);
  // TEST(test_optflow_track);
  // TEST(test_reproj_filter);
  // TEST(test_tracking);
  TEST(test_tsif);
}

#endif // AVS_UNITTESTS

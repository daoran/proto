#include <random>

#include <prototype/prototype.hpp>


using namespace proto;

struct calib_config_t {
  std::string target_file;
  std::string image_path;
  std::string pose_file;
  std::string preprocess_path;

  vec2_t resolution{0.0, 0.0};
  std::string camera_model;
  std::string distortion_model;
  vec4_t intrinsics = zeros(4, 1);
  vec4_t distortion = zeros(4, 1);
};

void print_usage() {
  const std::string usage = R"EOF(
Usage: calib_vicon_marker <calib_config.yaml>

The `calib_config.yaml` file is expected to have the following format:

  calib:
    target_file: "/data/aprilgrid_6x6.yaml"
    image_path: "/data/cam0/"
    pose_file: "/data/marker0/data.csv"
    preprocess_path: "/tmp/calib/mono"

  cam0:
    resolution: [752, 480]
    camera_model: "pinhole"
    distortion_model: "radtan"
    intrinsics: [459.176, 458.052, 369.511, 245.668]
    distortion: [-0.275281, 0.0671088, 0.000645568, -0.000474919]
)EOF";

  std::cout << usage << std::endl;
}

calib_config_t parse_config(const std::string &config_file) {
  config_t config{config_file};
  calib_config_t calib_config;

  parse(config, "calib.target_file", calib_config.target_file);
  parse(config, "calib.image_path", calib_config.image_path);
  parse(config, "calib.pose_file", calib_config.pose_file);
  parse(config, "calib.preprocess_path", calib_config.preprocess_path);

  parse(config, "cam0.resolution", calib_config.resolution);
  parse(config, "cam0.camera_model", calib_config.camera_model);
  parse(config, "cam0.distortion_model", calib_config.distortion_model);
  parse(config, "cam0.intrinsics", calib_config.intrinsics);
  parse(config, "cam0.distortion", calib_config.distortion);

  return calib_config;
}

int save_results(const std::string &save_path,
                 const vec2_t &resolution,
                 const pinhole_t &pinhole,
                 const radtan4_t &radtan,
                 const mat4_t &T_MC) {
  // Check if file is ok
  std::ofstream outfile(save_path);
  if (outfile.good() != true) {
    return -1;
  }

  // Save results
  const std::string indent = "  ";
  const std::string res = vec2str(resolution);
  const std::string intrinsics = arr2str(*pinhole.data, 4);
  const std::string distortion = arr2str(*radtan.data, 4);
  outfile << "cam0:" << std::endl;
  outfile << indent << "camera_model: \"pinhole\"" << std::endl;
  outfile << indent << "distortion_model: \"radtan\"" << std::endl;
  outfile << indent << "resolution: " << res << std::endl;
  outfile << indent << "intrinsics: " << intrinsics << std::endl;
  outfile << indent << "distortion: " << distortion << std::endl;
  outfile << std::endl;

  // Save marker to camera extrinsics
  outfile << "T_MC: " << std::endl;
  outfile << indent << "rows: 4" << std::endl;
  outfile << indent << "cols: 4" << std::endl;
  outfile << indent << "data: [" << std::endl;
  outfile << mat2str(T_MC, indent + indent) << std::endl;
  outfile << indent << "]" << std::endl;
  outfile << std::endl;

  // Finsh up
  outfile.close();

  return 0;
}

static int load_marker_poses(const calib_config_t &config,
                             const aprilgrids_t &grids,
                             const std::vector<double> &time_delays,
                             std::vector<long> &grid_ts,
                             mat4s_t &T_WM) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(config.pose_file, "r", &nb_rows);
  if (fp == nullptr) {
    LOG_ERROR("Failed to open [%s]!", config.pose_file.c_str());
    return -1;
  }

  // Check number of lines
  if (nb_rows <= 1) {
    LOG_ERROR("Marker pose file [%s] is empty?", config.pose_file.c_str());
    return -1;
  }

  // Check number of time delay values
  if ((nb_rows - 1) != (int) time_delays.size()) {
    LOG_ERROR("nb_rows != time_delays.size()");
    return -1;
  }

  // Skip first line
  skip_line(fp);

  // Parse data
  std::vector<long> timestamps;
  mat4s_t marker_poses;
  std::string str_format = "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf";
  for (int i = 0; i < nb_rows; i++) {
    // Parse line
    long ts = 0;
    double rx, ry, rz = 0.0;
    double qw, qx, qy, qz = 0.0;
    fscanf(fp, str_format.c_str(), &ts, &rx, &ry, &rz, &qw, &qx, &qy, &qz);

    // Form timestamps
    timestamps.push_back(ts + (long) (time_delays[i] * 1e9));

    // Form transform T_WM
    const vec3_t r_WM{rx, ry, rz};
    const quat_t q_WM{qw, qx, qy, qz};
    marker_poses.emplace_back(tf(q_WM, r_WM));
  }

  // Close file
  fclose(fp);

  // Interpolate transforms against aprilgrids
  for (size_t i = 0; i < grids.size(); i++) {
    grid_ts.push_back(grids[i].timestamp);
  }
  // interp_poses(timestamps, marker_poses, grid_ts, T_WM);
  closest_poses(timestamps, marker_poses, grid_ts, T_WM);

  return 0;
}

static int load_marker_poses(const calib_config_t &config,
                             const aprilgrids_t &grids,
                             std::vector<long> &grid_ts,
                             mat4s_t &T_WM,
                             const double time_delay=0.0) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(config.pose_file, "r", &nb_rows);
  if (fp == nullptr) {
    LOG_ERROR("Failed to open [%s]!", config.pose_file.c_str());
    return -1;
  }

  // Check number of lines
  if (nb_rows <= 1) {
    LOG_ERROR("Marker pose file [%s] is empty?", config.pose_file.c_str());
    return -1;
  }

  // Skip first line
  skip_line(fp);

  // Parse data
  std::vector<long> timestamps;
  mat4s_t marker_poses;
  std::string str_format = "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf";
  for (int i = 0; i < nb_rows; i++) {
    // Parse line
    long ts = 0;
    double rx, ry, rz = 0.0;
    double qw, qx, qy, qz = 0.0;
    fscanf(fp, str_format.c_str(), &ts, &rx, &ry, &rz, &qw, &qx, &qy, &qz);

    // Form timestamps
    timestamps.push_back(ts + (long) (time_delay * 1e9));

    // Form transform T_WM
    const vec3_t r_WM{rx, ry, rz};
    const quat_t q_WM{qw, qx, qy, qz};
    marker_poses.emplace_back(tf(q_WM, r_WM));
  }

  // Close file
  fclose(fp);

  // Interpolate transforms against aprilgrids
  for (size_t i = 0; i < grids.size(); i++) {
    grid_ts.push_back(grids[i].timestamp);
  }
  // interp_poses(timestamps, marker_poses, grid_ts, T_WM);
  closest_poses(timestamps, marker_poses, grid_ts, T_WM);

  return 0;
}

static int time_delay_search(const calib_config_t &config,
                             const aprilgrids_t &grids,
                             double &time_delay_result) {
  double time_delay_min = -0.1;
  double time_delay_max = 0.1;
  double time_delay_step = 0.0001;
  double time_delay = time_delay_min;

  double cost_best = DBL_MAX;
  double time_delay_best = time_delay_min;
  LOG_INFO("Evaluating time delay!");
  while (time_delay <= time_delay_max) {
    // Load marker poses
    mat4s_t T_WM;
    std::vector<long> timestamps;
    int retval = load_marker_poses(config,
                                   grids,
                                   timestamps,
                                   T_WM,
                                   time_delay);
    if (retval != 0) {
      LOG_ERROR("Failed to load marker poses!");
      return -1;
    }

    // Evaluate cost
    pinhole_t pinhole{config.intrinsics};
    radtan4_t radtan{config.distortion};
    const auto rpy_MC = deg2rad(vec3_t{-180.0, 0.0, -90.0});
    const auto C_MC = euler321ToRot(rpy_MC);
    mat4_t T_MC = tf(C_MC, zeros(3, 1));

    double cost = evaluate_vicon_marker_cost(grids,
                                             T_WM,
                                             pinhole,
                                             radtan,
                                             T_MC);
    if (cost_best > cost) {
      cost_best = cost;
      time_delay_best = time_delay;
    }
    time_delay += time_delay_step;
    std::cout << "." << std::flush;
  }
  std::cout << std::endl;
  LOG_INFO("Lowest cost: %.2e", cost_best);
  LOG_INFO("Time delay: %f", time_delay_best);

  time_delay_result = time_delay_best;

  return 0;
}

static std::vector<double> time_delay_search2(
    const calib_config_t &config,
    const aprilgrids_t &grids,
    const double time_delay_init) {
  // Get number of marker poses
  int nb_rows = 0;
  FILE *fp = file_open(config.pose_file, "r", &nb_rows);
  fclose(fp);
  nb_rows -= 1;  // Because of header

  // Initialize time delays
  std::vector<double> time_delays;
  for (int i = 0; i < nb_rows; i++) {
    time_delays.push_back(time_delay_init);
  }

  // Time delay search
  LOG_INFO("Evaluating time delay!");
  const int max_iter = 1000;
  double cost_best = DBL_MAX;
  std::vector<double> time_delays_best = time_delays;

  for (int i = 0; i < max_iter; i++) {
    // Load marker poses
    mat4s_t T_WM;
    std::vector<long> timestamps;
    int retval = load_marker_poses(config,
                                   grids,
                                   time_delays,
                                   timestamps,
                                   T_WM);
    if (retval != 0) {
      FATAL("Failed to load marker poses!");
    }

    // Evaluate cost
    // -- Setup
    pinhole_t pinhole{config.intrinsics};
    radtan4_t radtan{config.distortion};
    const auto rpy_MC = deg2rad(vec3_t{-180.0, 0.0, -90.0});
    const auto C_MC = euler321ToRot(rpy_MC);
    mat4_t T_MC = tf(C_MC, zeros(3, 1));
    // -- Evaluate
    double cost = evaluate_vicon_marker_cost(grids,
                                             T_WM,
                                             pinhole,
                                             radtan,
                                             T_MC);

    // Keep best results
    if (cost_best > cost) {
      cost_best = cost;
      time_delays_best = time_delays;
    } else {
      time_delays = time_delays_best;
    }
    std::cout << "Iter: " << i << "\t";
    std::cout << "Cost: " << cost << "\t";
    std::cout << "Best Cost: " << cost_best << "\n";

    // Perturbe the time delays a little
    std::random_device rd;     // Only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // Random-number engine used (Mersenne-Twister in this case)
    std::uniform_real_distribution<double> random_sample(0, 0.2);
    std::uniform_int_distribution<int> random_element(0, nb_rows - 1);
    std::uniform_real_distribution<double> random_value(-0.1, 0.1);
    for (int i = 0; i < (int) (time_delays.size() * random_sample(rng)); i++) {
      time_delays[random_element(rng)] += random_value(rng);
    }
  }
  std::cout << "Best Cost: " << cost_best << std::endl;
  std::cout << "Best result: " << std::endl;
  for (const auto &t: time_delays) {
    std::cout << t << std::endl;
  }
  std::cout << std::endl;

  return time_delays_best;
}


static int get_image_paths(const std::string &image_dir,
                           std::vector<std::string> &image_paths) {
  // Check image dir
  if (dir_exists(image_dir) == false) {
    LOG_ERROR("Image dir [%s] does not exist!", image_dir.c_str());
    return -1;
  }

  // Get image paths
  if (list_dir(image_dir, image_paths) != 0) {
    LOG_ERROR("Failed to traverse dir [%s]!", image_dir.c_str());
    return -1;
  }
  std::sort(image_paths.begin(), image_paths.end());

  return 0;
}

static cv::Mat project_aprilgrid(
    const aprilgrid_t &grid,
    const camera_geometry_t<pinhole_t, radtan4_t> &camera,
    const cv::Mat &image,
    const mat4_t &T_WM,
    const mat4_t &T_MC,
    const mat4_t &T_WF) {
  const auto T_MW = T_WM.inverse();
  const auto T_CM = T_MC.inverse();

  // Make an RGB version of the input image
  cv::Mat image_rgb = gray2rgb(image);

  // Project AprilGrid
  for (const auto &tag_id : grid.ids) {
    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(grid, tag_id, object_points) != 0) {
      FATAL("Failed to calculate AprilGrid object points!");
    }

    // Project corner points
    for (size_t j = 0; j < 4; j++) {
      const auto hp_F = object_points[j].homogeneous();
      const auto hp_C = T_CM * T_MW * T_WF * hp_F;
      const auto pixel = camera_geometry_project(camera, hp_C.head(3));

      const cv::Point2f p(pixel(0), pixel(1));
      cv::circle(image_rgb,              // Target image
                 p,                      // Center
                 3,                      // Radius
                 cv::Scalar{0, 0, 255},  // Colour
                 CV_FILLED,              // Thickness
                 8);                     // Line type
    }
  }

  return image_rgb;
}

static int validate(const calib_config_t &config,
                    const calib_target_t &target,
                    const std::vector<long> &timestamps,
                    const mat4s_t &T_WM,
                    const mat4_t &T_MC,
                    const mat4_t &T_WF) {
  // Get camera image paths
  std::vector<std::string> image_paths;
  if (get_image_paths(config.image_path, image_paths) != 0) {
    return -1;
  }

  // Detect AprilGrid
  LOG_INFO("Processing images:");
  aprilgrid_detector_t detector;
  const pinhole_t pinhole{config.intrinsics};
  const radtan4_t radtan{config.distortion};
  const camera_geometry_t<pinhole_t, radtan4_t> camera(pinhole, radtan);

  int pose_idx = 0;
  std::deque<long int> ts_queue(timestamps.begin(), timestamps.end());
  for (size_t i = 0; i < image_paths.size(); i++) {
    // Grab timestamp from file name
    std::string output = image_paths[i];
    const size_t last_slash_idx = output.find_last_of("\\/");
    if (std::string::npos != last_slash_idx) {
      output.erase(0, last_slash_idx + 1);
    }

    // Check to see if image timestamp is among the timestamps we optimized against
    const long image_ts = std::stol(output);
    bool found = false;
    for (const auto &ts : ts_queue) {
      if (image_ts < ts) {
        ts_queue.pop_front();
      } else if (image_ts == ts) {
        ts_queue.pop_front();
        found = true;
        break;
      }
    }
    if (found) {

    // Detect AprilGrid
    const auto image_path = paths_combine(config.image_path, image_paths[i]);
    const cv::Mat image = cv::imread(image_path);
    aprilgrid_t grid{(long) i,
                     target.tag_rows,
                     target.tag_cols,
                     target.tag_size,
                     target.tag_spacing};
    aprilgrid_detect(grid, detector, image);

    // Project AprilGrid
    const auto image_rgb = project_aprilgrid(grid,
                                             camera,
                                             image,
                                             T_WM[pose_idx],
                                             T_MC,
                                             T_WF);
    pose_idx++;
    cv::imshow("Validate:", image_rgb);
    cv::waitKey(1);
    }
  }

  // Destroy all opencv windows
  cv::destroyAllWindows();

  return 0;
}

int main(int argc, char *argv[]) {
  // Parse command line arguments
  if (argc != 2) {
    print_usage();
    return -1;
  }

  // Parse calib config file
  const std::string config_file{argv[1]};
  calib_config_t config = parse_config(config_file);

  // Load calibration target
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config.target_file) != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", config.target_file.c_str());
    return -1;
  }

  // Preprocess calibration data
  int retval = preprocess_camera_data(calib_target,
                                      config.image_path,
                                      pinhole_K(config.intrinsics),
                                      config.distortion,
                                      config.preprocess_path);
  if (retval != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  // Load camera data
  std::vector<aprilgrid_t> grids;
  if (load_camera_calib_data(config.preprocess_path, grids) != 0) {
    LOG_ERROR("Failed to load camera calibration data!");
    return -1;
  }

  // // Search time delay
  // std::vector<double> time_delays;
  // if (file_exists("time_delays.csv")) {
  //   // -- Parse time delays
  //   int nb_rows = 0;
  //   FILE *fp = file_open("time_delays.csv", "r", &nb_rows);
  //   for (int i = 0; i < nb_rows; i++) {
  //     double time_delay = 0.0;
  //     fscanf(fp, "%lf", &time_delay);
  //     time_delays.push_back(time_delay);
  //   }
  //   fclose(fp);
  //
  // } else {
  //   // -- First stage: Get general time delay constant
  //   double time_delay = 0.0;
  //   time_delay_search(config, grids, time_delay);
  //   // -- Second stage Refine individual time delays
  //   time_delays = time_delay_search2(config, grids, time_delay);
  //   // -- Output results
  //   FILE *fp = fopen("time_delays.csv", "w");
  //   for (const auto &delay: time_delays) {
  //     fprintf(fp, "%f\n", delay);
  //   }
  //   fclose(fp);
  // }

  // // Load marker poses
  // mat4s_t T_WM;
  // std::vector<long> timestamps;
  // if (load_marker_poses(config, grids, time_delays, timestamps, T_WM) != 0) {
  //   LOG_ERROR("Failed to load marker poses!");
  //   return -1;
  // }

  // Load marker poses
  mat4s_t T_WM;
  std::vector<long> timestamps;
  if (load_marker_poses(config, grids, timestamps, T_WM) != 0) {
    LOG_ERROR("Failed to load marker poses!");
    return -1;
  }

  // Save marker poses
  FILE *fp = fopen("/tmp/marker0_timestamps.csv", "w");
  fprintf(fp, "timestamp,qw,qx,qy,qz,rx,ry,rz\n");
  for (size_t i = 0; i < timestamps.size(); i++) {
    const vec3_t r = tf_trans(T_WM[i]);
    const quat_t q = quat_t{tf_rot(T_WM[i])};
    fprintf(fp, "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
            timestamps[i],
            q.w(), q.x(), q.y(), q.z(),
            r(0), r(1), r(2));
  }
  fclose(fp);

  // Calibrate vicon marker
  LOG_INFO("Calibrating vicon marker!");
  pinhole_t pinhole{config.intrinsics};
  radtan4_t radtan{config.distortion};
  const auto rpy_MC = deg2rad(vec3_t{-180.0, 0.0, -90.0});
  const auto C_MC = euler321ToRot(rpy_MC);
  mat4_t T_MC = tf(C_MC, 1e-5 * ones(3, 1));
  mat4_t T_WF = T_WM[0] * T_MC * grids[0].T_CF;

  // -- Solve
  retval = calib_vicon_marker_solve(grids,
                                    T_WM,
                                    pinhole,
                                    radtan,
                                    T_MC,
                                    T_WF);
  if (retval != 0) {
    LOG_ERROR("Failed to calibrate camera data!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  std::cout << pinhole << std::endl;
  std::cout << radtan << std::endl;
  std::cout << "T_MC:\n" << T_MC << std::endl << std::endl;
  std::cout << "T_WF:\n" << T_WF << std::endl << std::endl;

  // Save results
  const std::string save_path{"./calib_results.yaml"};
  LOG_INFO("Saving optimization results to [%s]", save_path.c_str());
  if (save_results(save_path, config.resolution, pinhole, radtan, T_MC) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", save_path.c_str());
    return -1;
  }

  // Validate
  config.intrinsics = vec4_t{pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy};
  config.distortion = vec4_t{radtan.k1, radtan.k2, radtan.p1, radtan.p2};
  validate(config, calib_target, timestamps, T_WM, T_MC, T_WF);

  return 0;
}

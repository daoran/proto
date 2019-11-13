#include "proto/proto.hpp"
#include "proto/calib/calib_vicon_marker.hpp"
#include "proto/ros/node.hpp"
#include "proto/ros/bag.hpp"

using namespace proto;

std::string basename(const std::string &path) {
  auto output = path;
  const size_t last_slash_idx = output.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    output.erase(0, last_slash_idx + 1);
  }

  return output;
}

void process_rosbag(const std::string &rosbag_path,
										const std::string &out_path,
										const std::string &cam0_topic,
										const std::string &body0_topic,
										const std::string &target0_topic) {
	// Check whether ros topics are in bag
  std::vector<std::string> target_topics;
  target_topics.push_back(cam0_topic);
  target_topics.push_back(body0_topic);
  target_topics.push_back(target0_topic);
  if (check_ros_topics(rosbag_path, target_topics) == false) {
    FATAL("Failed to create dir [%s]", out_path.c_str());
  }

  // Check output dir
  std::cout << "[" << out_path << "]" << std::endl;
  if (dir_exists(out_path) == false) {
    if (dir_create(out_path) != 0) {
      FATAL("Failed to create dir [%s]", out_path.c_str());
    }
  }

  // Prepare data files
  const auto cam0_output_path = out_path + "/cam0";
  const auto body0_output_path = out_path + "/body0";
  const auto target0_output_path = out_path + "/target0";
  auto cam0_csv = camera_init_output_file(cam0_output_path);
  auto body0_csv = pose_init_output_file(body0_output_path);
  auto target0_csv = pose_init_output_file(target0_output_path);

  // Process ROS bag
  LOG_INFO("Processing ROS bag [%s]", rosbag_path.c_str());
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  rosbag::View bag_view(bag);
  size_t msg_idx = 0;

  for (const auto &msg : bag_view) {
    // Print progress
    print_progress((double) msg_idx / bag_view.size());
    msg_idx++;

    // Process camera data
    if (msg.getTopic() == cam0_topic) {
      image_message_handler(
        msg,
        cam0_output_path + "/data/",
        cam0_csv
      );
    }

    // Process body data
    if (msg.getTopic() == body0_topic) {
      pose_message_handler(
        msg,
        body0_output_path + "/data/",
        body0_csv
      );
    }

    // Process target data
    if (msg.getTopic() == target0_topic) {
      pose_message_handler(
        msg,
        target0_output_path + "/data/",
        target0_csv
      );
    }
  }

  // Clean up rosbag
  print_progress(1.0);
  bag.close();
}

static aprilgrids_t load_aprilgrids(const std::string &dir_path) {
  std::vector<std::string> csv_files;
  if (list_dir(dir_path, csv_files) != 0) {
    FATAL("Failed to list dir [%s]!", dir_path.c_str());
  }
  sort(csv_files.begin(), csv_files.end());

  aprilgrids_t grids;
  for (const auto &grid_csv : csv_files) {
    const auto csv_path = dir_path + "/" + grid_csv;
    aprilgrid_t grid;
    if (aprilgrid_load(grid, csv_path) != 0) {
      FATAL("Failed to load AprilGrid [%s]!", grid_csv.c_str());
    }

    if (grid.detected) {
      grids.push_back(grid);
    }
  }

  return grids;
}

static void load_body_poses(const std::string &fpath,
                            timestamps_t &timestamps,
                            mat4s_t &poses) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(fpath.c_str(), "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", fpath.c_str());
  }

  // Create format string
  std::string str_format;
  str_format = "%ld,";               // Timestamp[ns]
  str_format += "%lf,%lf,%lf,%lf,";  // Quaternion
  str_format += "%lf,%lf,%lf";       // Position

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double qw, qx, qy, qz = 0.0;
    double px, py, pz = 0.0;
    int retval = fscanf(
      fp,
      str_format.c_str(),
      &ts,
      &qw, &qx, &qy, &qz,
      &px, &py, &pz
    );
    if (retval != 8) {
      FATAL("Failed to parse line in [%s:%d]", fpath.c_str(), i);
    }

    // Record
    timestamps.push_back(ts);
    quat_t q{qw, qx, qy, qz};
    vec3_t r{px, py, pz};
    poses.push_back(tf(q, r));
  }
}

static mat4_t load_fiducial_pose(const std::string &fpath) {
  mat4_t T_WF;

  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(fpath.c_str(), "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", fpath.c_str());
  }

  // Create format string
  std::string str_format;
  str_format = "%ld,";               // Timestamp[ns]
  str_format += "%lf,%lf,%lf,%lf,";  // Quaternion
  str_format += "%lf,%lf,%lf";       // Position

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double qw, qx, qy, qz = 0.0;
    double px, py, pz = 0.0;
    int retval = fscanf(
      fp,
      str_format.c_str(),
      &ts,
      &qw, &qx, &qy, &qz,
      &px, &py, &pz
    );
    if (retval != 8) {
      FATAL("Failed to parse line in [%s:%d]", fpath.c_str(), i);
    }

    // Just need 1 pose
    quat_t q{qw, qx, qy, qz};
    vec3_t r{px, py, pz};
    return tf(q, r);
  }
}

mat4_t lerp_pose(const timestamp_t &t0,
                 const mat4_t &pose0,
                 const timestamp_t &t1,
                 const mat4_t &pose1,
                 const timestamp_t &t_lerp) {
  // Calculate alpha
  const double numerator = (t_lerp - t0) * 1e-9;
  const double denominator = (t1 - t0) * 1e-9;
  const double alpha = numerator / denominator;

  // Decompose start pose
  const vec3_t trans0 = tf_trans(pose0);
  const quat_t quat0{tf_rot(pose0)};

  // Decompose end pose
  const vec3_t trans1 = tf_trans(pose1);
  const quat_t quat1{tf_rot(pose1)};

  // Interpolate translation and rotation
  const auto trans_interp = lerp(trans0, trans1, alpha);
  const auto quat_interp = quat1.slerp(alpha, quat0);

  return tf(quat_interp, trans_interp);
}

void lerp_body_poses(const aprilgrids_t &grids,
                     const timestamps_t &body_timestamps,
                     const mat4s_t &body_poses,
                     aprilgrids_t &lerped_grids,
                     mat4s_t &lerped_poses) {
  // Make sure AprilGrids are between body poses else we can't lerp poses
  timestamps_t grid_timestamps;
  for (const auto &grid : grids) {
    if (grid.timestamp > body_timestamps.front() &&
        grid.timestamp < body_timestamps.back()) {
      lerped_grids.push_back(grid);
      grid_timestamps.push_back(grid.timestamp);
    }
  }

  // Lerp body poses using AprilGrid timestamps
  assert(body_poses.size() == body_timestamps.size());
  assert(body_timestamps.front() < grid_timestamps.front());
  timestamp_t t0 = 0;
  mat4_t pose0 = I(4);
  timestamp_t t1 = 0;
  mat4_t pose1 = I(4);

  size_t grid_idx = 0;
  for (size_t i = 0; i < body_timestamps.size(); i++) {
    // Make sure we're not going out of bounds
    if (grid_idx > (grid_timestamps.size() - 1)) {
      break;
    }

    // Get time now and desired lerp time
    const auto t_now = body_timestamps[i];
    const auto t_lerp = grid_timestamps[grid_idx];

    // Update t0
    if (t_now < t_lerp) {
      t0 = t_now;
      pose0 = body_poses[i];
    }

    // Update t1
    if (t_now > t_lerp) {
      // Lerp
      t1 = t_now;
      pose1 = body_poses[i];
      const auto pose = lerp_pose(t0, pose0, t1, pose1, t_lerp);
      lerped_poses.push_back(pose);
      grid_idx++;

      // Reset
      t0 = t_now;
      pose0 = body_poses[i];
      t1 = 0;
      pose1 = I(4);
    }
  }
}

void save_results(const std::string &output_path,
                  const aprilgrid_t &grid,
                  const pinhole_t &pinhole,
                  const radtan4_t &radtan,
                  const mat4_t &T_WF,
                  const mat4_t &T_MC) {
  LOG_INFO("Saving results to [%s]!", output_path.c_str());
  FILE *fp = fopen(output_path.c_str(), "w");

  // Aprilgrid parameters
  fprintf(fp, "calib_target:\n");
  fprintf(fp, "  target_type: \"aprilgrid\"\n");
  fprintf(fp, "  tag_rows: %d\n", grid.tag_rows);
  fprintf(fp, "  tag_cols: %d\n", grid.tag_cols);
  fprintf(fp, "  tag_size: %f\n", grid.tag_size);
  fprintf(fp, "  tag_spacing: %f\n", grid.tag_spacing);
  fprintf(fp, "\n");

  // Camera parameters
  fprintf(fp, "cam0:\n");
  fprintf(fp, "  camera_model: \"pinhole\"\n");
  fprintf(fp, "  distortion_model: \"radtan\"\n");
  fprintf(fp, "  intrinsics: ");
  fprintf(fp, "[");
  fprintf(fp, "%lf, ", pinhole.fx);
  fprintf(fp, "%lf, ", pinhole.fy);
  fprintf(fp, "%lf, ", pinhole.cx);
  fprintf(fp, "%lf", pinhole.cy);
  fprintf(fp, "]\n");
  fprintf(fp, "  distortion: ");
  fprintf(fp, "[");
  fprintf(fp, "%lf, ", radtan.k1);
  fprintf(fp, "%lf, ", radtan.k2);
  fprintf(fp, "%lf, ", radtan.p1);
  fprintf(fp, "%lf", radtan.p2);
  fprintf(fp, "]\n");
  fprintf(fp, "\n");

  // T_WF
  fprintf(fp, "T_WF:\n");
  fprintf(fp, "  rows: 4\n");
  fprintf(fp, "  cols: 4\n");
  fprintf(fp, "  data: [\n");
  fprintf(fp, "    ");
  fprintf(fp, "%lf, ", T_WF(0, 0));
  fprintf(fp, "%lf, ", T_WF(0, 1));
  fprintf(fp, "%lf, ", T_WF(0, 2));
  fprintf(fp, "%lf,\n", T_WF(0, 3));
  fprintf(fp, "    ");
  fprintf(fp, "%lf, ", T_WF(1, 0));
  fprintf(fp, "%lf, ", T_WF(1, 1));
  fprintf(fp, "%lf, ", T_WF(1, 2));
  fprintf(fp, "%lf,\n", T_WF(1, 3));
  fprintf(fp, "    ");
  fprintf(fp, "%lf, ", T_WF(2, 0));
  fprintf(fp, "%lf, ", T_WF(2, 1));
  fprintf(fp, "%lf, ", T_WF(2, 2));
  fprintf(fp, "%lf,\n", T_WF(2, 3));
  fprintf(fp, "    ");
  fprintf(fp, "%lf, ", T_WF(3, 0));
  fprintf(fp, "%lf, ", T_WF(3, 1));
  fprintf(fp, "%lf, ", T_WF(3, 2));
  fprintf(fp, "%lf\n", T_WF(3, 3));
  fprintf(fp, "  ]\n");
  fprintf(fp, "\n");

  // T_MC
  fprintf(fp, "T_MC:\n");
  fprintf(fp, "  rows: 4\n");
  fprintf(fp, "  cols: 4\n");
  fprintf(fp, "  data: [\n");
  fprintf(fp, "    ");
  fprintf(fp, "%lf, ", T_MC(0, 0));
  fprintf(fp, "%lf, ", T_MC(0, 1));
  fprintf(fp, "%lf, ", T_MC(0, 2));
  fprintf(fp, "%lf,\n", T_MC(0, 3));
  fprintf(fp, "    ");
  fprintf(fp, "%lf, ", T_MC(1, 0));
  fprintf(fp, "%lf, ", T_MC(1, 1));
  fprintf(fp, "%lf, ", T_MC(1, 2));
  fprintf(fp, "%lf,\n", T_MC(1, 3));
  fprintf(fp, "    ");
  fprintf(fp, "%lf, ", T_MC(2, 0));
  fprintf(fp, "%lf, ", T_MC(2, 1));
  fprintf(fp, "%lf, ", T_MC(2, 2));
  fprintf(fp, "%lf,\n", T_MC(2, 3));
  fprintf(fp, "    ");
  fprintf(fp, "%lf, ", T_MC(3, 0));
  fprintf(fp, "%lf, ", T_MC(3, 1));
  fprintf(fp, "%lf, ", T_MC(3, 2));
  fprintf(fp, "%lf\n", T_MC(3, 3));
  fprintf(fp, "  ]\n");
  fprintf(fp, "\n");

  fclose(fp);
}

int main(int argc, char *argv[]) {
  // Setup ROS Node
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  const ros::NodeHandle ros_nh;
  std::string rosbag_path;
  std::string config_file;
  std::string cam0_topic;
  std::string body0_topic;
  std::string target0_topic;
  ROS_PARAM(ros_nh, node_name + "/rosbag", rosbag_path);
  ROS_PARAM(ros_nh, node_name + "/config_file", config_file);
  ROS_PARAM(ros_nh, node_name + "/cam0_topic", cam0_topic);
  ROS_PARAM(ros_nh, node_name + "/body0_topic", body0_topic);
  ROS_PARAM(ros_nh, node_name + "/target0_topic", target0_topic);

  std::string rosbag_validate_path;
  std::string validate_path = "/tmp/vicon_validation";
  ROS_PARAM(ros_nh, node_name + "/rosbag_validate", rosbag_validate_path);

  // Parse calibration target params
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
    FATAL("Failed to parse calib file [%s]!", config_file.c_str());
  }

	// Parse config file
	std::string data_path;
	std::string calib_results_path;
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_fpath", calib_results_path);

	// Process rosbag
	process_rosbag(rosbag_path,
	               data_path,
	               cam0_topic,
	               body0_topic,
	               target0_topic);

  // Calibrate camera intrinsics
  if (calib_camera_solve(config_file) != 0) {
    FATAL("Failed to calibrate camera!");
  }

  // Load camera calibration results
  vec2_t resolution;
  vec4_t intrinsics;
  vec4_t distortion;
  config_t calib_results{calib_results_path};
  parse(calib_results, "cam0.resolution", resolution);
  parse(calib_results, "cam0.intrinsics", intrinsics);
  parse(calib_results, "cam0.distortion", distortion);

  // -- April Grid
  const auto grid0_path = data_path + "/grid0/cam0/data";
  aprilgrids_t aprilgrids = load_aprilgrids(grid0_path);

  // -- Camera intrinsics and distortion
  pinhole_t pinhole{intrinsics};
  radtan4_t radtan{distortion};

  // -- Vicon marker pose
  const auto body0_csv_path = data_path + "/body0/data.csv";
  timestamps_t body_timestamps;
  mat4s_t body_poses;
  load_body_poses(body0_csv_path, body_timestamps, body_poses);

  aprilgrids_t lerped_grids;
  mat4s_t lerped_body_poses;
  lerp_body_poses(aprilgrids, body_timestamps, body_poses,
                  lerped_grids, lerped_body_poses);

  // -- Vicon Marker to Camera transform
  // const vec3_t euler{-90.0, 0.0, -90.0};
  // const mat3_t C = euler321(deg2rad(euler));
  // mat4_t T_MC = tf(C, zeros(3, 1));
  mat4_t T_MC;
  T_MC << 0.0, 0.0, 1.0, 0.10,
          -1.0, 0.0, 0.0, 0.07,
          0.0, -1.0, -0.0, -0.15,
          0.0, 0.0, 0.0, 1.0;


  // -- Fiducial target pose
  const auto target0_csv_path = data_path + "/target0/data.csv";
  mat4_t T_WF = load_fiducial_pose(target0_csv_path);
  // T_WF(0, 3) += 0.07;
  // T_WF(1, 3) += 0.07;
  print_matrix("Initial T_WF", T_WF);

  // Save vicon calibration problem
  calib_vicon_marker_solve(lerped_grids,
                           pinhole,
                           radtan,
                           lerped_body_poses,
                           T_MC,
                           T_WF);

  mat4s_t T_CF;
  {
    const mat4_t T_CM = T_MC.inverse();
    for (size_t i = 0; i < lerped_grids.size(); i++) {
      const mat4_t T_MW = lerped_body_poses[i].inverse();
      T_CF.emplace_back(T_CM * T_MW * T_WF);
    }
  }

  // Show and save results
  std::cout << std::endl;
  std::cout << "Calibration Results:" << std::endl;
  std::cout << "----------------------------------------" << std::endl;
  calib_camera_stats<pinhole_radtan4_residual_t>(
    lerped_grids,
    *pinhole.data,
    *radtan.data,
    T_CF,
    ""
  );
  std::cout << std::endl;
  std::cout << "Pinhole:\n" << pinhole << std::endl;
  std::cout << "Radtan:\n" << radtan << std::endl;
  print_matrix("T_WF", T_WF);
  print_matrix("T_WM", lerped_body_poses[0]);
  print_matrix("T_MC", T_MC);
  save_results("/tmp/vicon_calib.yaml",
               lerped_grids[0],
               pinhole,
               radtan,
               T_WF,
               T_MC);

  // Record poses
  {
    FILE *fp = fopen("/tmp/poses.csv", "w");
    for (const auto &pose: lerped_body_poses) {
      const auto q = tf_quat(pose);
      const auto r = tf_trans(pose);
      fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
      fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
      fprintf(fp, "\n");
    }
    fclose(fp);
  }

  {
    FILE *fp = fopen("/tmp/T_WF.csv", "w");
    const auto q = tf_quat(T_WF);
    const auto r = tf_trans(T_WF);
    fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
    fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
    fprintf(fp, "\n");
    fclose(fp);
  }

  {
    FILE *fp = fopen("/tmp/T_MC.csv", "w");
    const auto q = tf_quat(T_MC);
    const auto r = tf_trans(T_MC);
    fprintf(fp, "%lf,%lf,%lf,%lf,", q.w(), q.x(), q.y(), q.z());
    fprintf(fp, "%lf,%lf,%lf", r(0), r(1), r(2));
    fprintf(fp, "\n");
    fclose(fp);
  }

  // Process validation ROS bag
	process_rosbag(rosbag_validate_path,
	               validate_path,
	               cam0_topic,
	               body0_topic,
	               target0_topic);

  // Detect Aprilgrids in validation set
  {
    const auto cam0_path = validate_path + "/cam0/data";
    const auto grid_path = validate_path + "/grid0/cam0/data";

    std::vector<std::string> image_paths;
    if (list_dir(cam0_path, image_paths) != 0) {
      FATAL("Failed to list dir [%s]!", cam0_path.c_str());
    }
    sort(image_paths.begin(), image_paths.end());

    // Detect AprilGrid
    aprilgrid_detector_t detector;
    for (size_t i = 0; i < image_paths.size(); i++) {
      // -- Create output file path
      auto output_file = basename(image_paths[i]);
      const timestamp_t ts = std::stoull(output_file);
      output_file = remove_ext(output_file);
      output_file += ".csv";
      const auto save_path = paths_combine(grid_path, output_file);

      // -- Setup AprilGrid
      const int tag_rows = calib_target.tag_rows;
      const int tag_cols = calib_target.tag_cols;
      const double tag_size = calib_target.tag_size;
      const double tag_spacing = calib_target.tag_spacing;
      aprilgrid_t grid{ts, tag_rows, tag_cols, tag_size, tag_spacing};

      // -- Skip if already preprocessed
      if (file_exists(save_path) && aprilgrid_load(grid, save_path) == 0) {
        continue;
      } else {
        // Reset AprilGrid
        grid = aprilgrid_t{ts, tag_rows, tag_cols, tag_size, tag_spacing};
      }

      // -- Detect
      const auto image_path = paths_combine(cam0_path, image_paths[i]);
      const cv::Mat image = cv::imread(image_path);
      aprilgrid_detect(grid, detector, image);
      grid.timestamp = ts;

      // -- Save AprilGrid
      if (aprilgrid_save(grid, save_path) != 0) {
        return -1;
      }
    }
  }

  {
    // -- Camera
    const auto cam0_path = validate_path + "/cam0/data";
    std::vector<std::string> cam0_files;
    if (list_dir(cam0_path, cam0_files) != 0) {
      FATAL("Failed to list dir [%s]!", cam0_path.c_str());
    }
    sort(cam0_files.begin(), cam0_files.end());

    // -- Aprilgrids
    const auto grids_path = validate_path + "/grid0/cam0/data";
    aprilgrids_t aprilgrids = load_aprilgrids(grids_path);

    // -- Vicon marker pose
    const auto body0_csv_path = validate_path + "/body0/data.csv";
    timestamps_t body_timestamps;
    mat4s_t body_poses;
    load_body_poses(body0_csv_path, body_timestamps, body_poses);

    aprilgrids_t lerped_grids;
    mat4s_t lerped_body_poses;
    lerp_body_poses(aprilgrids, body_timestamps, body_poses,
                    lerped_grids, lerped_body_poses);

    mat3_t K = pinhole_K(pinhole);
    vec4_t D{radtan.k1, radtan.k2, radtan.p1, radtan.p2};

    // -- Loop over validation images
    size_t pose_idx = 0;
    for (const auto &image_file : cam0_files) {
      LOG_INFO("Image [%s]", image_file.c_str());
      auto image = cv::imread(paths_combine(cam0_path, image_file));
      const auto img_w = image.cols;
      const auto img_h = image.rows;

      // Predict where aprilgrid points should be
      const mat4_t T_WM = lerped_body_poses[pose_idx];
      const mat4_t T_WC = T_WM * T_MC;
      const mat4_t T_CW = T_WC.inverse();

      aprilgrid_t grid;
      grid.tag_rows = calib_target.tag_rows;
      grid.tag_cols = calib_target.tag_cols;
      grid.tag_size = calib_target.tag_size;
      grid.tag_spacing = calib_target.tag_spacing;

      vec3s_t object_points;
      vec2s_t image_points;
      aprilgrid_object_points(grid, object_points);
      for (const auto &p_F : object_points) {
        const vec3_t p_C = (T_CW * T_WF * p_F.homogeneous()).head(3);
        if (p_C(2) < 0) {
          break;
        }

        const vec2_t img_pt = pinhole_radtan4_project(K, D, p_C);
        const bool x_ok = (img_pt(0) > 0 && img_pt(0) < img_w);
        const bool y_ok = (img_pt(1) > 0 && img_pt(1) < img_h);
        if (x_ok && y_ok) {
          image_points.push_back(img_pt);
        }
      }

      // Draw on image
      for (const auto &img_pt : image_points) {
        cv::Point2f p(img_pt(0), img_pt(1));
        cv::circle(image, p, 3, cv::Scalar(0, 0, 255), -1);
      }

      cv::imshow("Image", image);
      cv::waitKey(0);
      pose_idx++;
    }
  }

  return 0;
}

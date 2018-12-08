#include <libgen.h>
#include <prototype/prototype.hpp>

using namespace prototype;

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
  std::ofstream outfile(save_path);

  // Check if file is ok
  if (outfile.good() != true) {
    return -1;
  }

  // Save results
  const std::string indent = "  ";
  {
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
  }

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

static std::vector<long> get_timestamps(const std::string &file_path,
                                        const bool header=false) {
  std::vector<long> timestamps;

  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    FATAL("Cannot open file [%s]!", file_path.c_str());
  }

  // Obtain number of rows and cols
  int nb_rows = csvrows(file_path);

  // Header line?
  std::string line;
  if (header) {
    std::getline(infile, line);
    nb_rows -= 1;
  }

  // load data
  size_t line_no = 0;
  std::string element;
  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    // load data row
    for (int i = 0; i < 1; i++) {
      std::getline(ss, element, ',');
      timestamps.push_back(atol(element.c_str()));
    }

    line_no++;
  }

  return timestamps;
}

static int load_marker_poses(const calib_config_t &config,
                             const aprilgrids_t &aprilgrids,
                             std::vector<long> &timestamps_filtered,
                             mat4s_t &T_WM) {
  // Load marker pose data
  // -- First get timestamps
  const auto timestamps = get_timestamps(config.pose_file, true);

  // -- Load marker pose data
  matx_t pose_data;
  if (csv2mat(config.pose_file, true, pose_data) != 0) {
    LOG_ERROR("Failed to load pose data [%s]!", config.pose_file.c_str());
    return -1;
  }

  // -- Only keep marker poses where AprilGrid is detected
  size_t grid_idx = 0;
  size_t ts_idx = 0;

  while (true) {
    const auto &grid = aprilgrids[grid_idx];
    const long ts = timestamps[ts_idx];

    if (grid.timestamp == ts) {
      timestamps_filtered.push_back(ts);

      // Translation
      const double rx = pose_data(ts_idx, 1);
      const double ry = pose_data(ts_idx, 2);
      const double rz = pose_data(ts_idx, 3);
      const vec3_t r_WM{rx, ry, rz};
      // Rotation
      const double qw = pose_data(ts_idx, 4);
      const double qx = pose_data(ts_idx, 5);
      const double qy = pose_data(ts_idx, 6);
      const double qz = pose_data(ts_idx, 7);
      const quat_t q_WM{qw, qx, qy, qz};
      // Add transform T_WM
      T_WM.emplace_back(tf(q_WM, r_WM));

      // Update indicies
      grid_idx++;
      ts_idx++;

    } else if (grid.timestamp > ts) {
      ts_idx++;
    } else if (grid.timestamp < ts) {
      grid_idx++;
    }

    // Reach the end of the line?
    if (grid_idx == aprilgrids.size() || ts_idx == timestamps.size()) {
      break;
    }
  }

  std::cout << aprilgrids.size() << std::endl;
  std::cout << T_WM.size() << std::endl;
  exit(0);

  return 0;
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
    const auto T_MW = T_WM.inverse();
    const auto T_CM = T_MC.inverse();
    for (size_t j = 0; j < 4; j++) {
      const auto hp_F = object_points[j].homogeneous();
      const auto hp_C = T_CM * T_MW * T_WF * hp_F;
      const auto pixel = camera_geometry_project(camera, hp_C.head(3));

      const cv::Point2f p(pixel(0), pixel(1));
      cv::circle(image_rgb,              // Target image
                  p,                      // Center
                  1,                      // Radius
                  cv::Scalar{0, 0, 255},  // Colour
                  CV_FILLED,              // Thickness
                  8);                     // Line type
    }
  }

  return image_rgb;
}

static std::string basename(const std::string &path) {
  auto output = path;
  const size_t last_slash_idx = output.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    output.erase(0, last_slash_idx + 1);
  }

  return output;
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
  const mat3_t cam_K = pinhole_K(config.intrinsics);
  const vec4_t cam_D = config.distortion;
  const pinhole_t pinhole{config.intrinsics};
  const radtan4_t radtan{config.distortion};
  const camera_geometry_t<pinhole_t, radtan4_t> camera(pinhole, radtan);

  for (size_t i = 1500; i < image_paths.size(); i++) {
    // print_progress((double) i / image_paths.size());

    // Grab timestamp from file name and check if against calibrated
    std::string output = image_paths[i];
    const size_t last_slash_idx = output.find_last_of("\\/");
    if (std::string::npos != last_slash_idx) {
      output.erase(0, last_slash_idx + 1);
    }
    const long ts = std::stol(output);
    if (std::count(timestamps.begin(), timestamps.end(), ts) == 0) {
      std::cout << "skipping" << std::endl;
      continue;
    }

    // Detect AprilGrid
    const auto image_path = paths_combine(config.image_path, image_paths[i]);
    const cv::Mat image = cv::imread(image_path);
    aprilgrid_t grid{(long) i,
                     target.tag_rows,
                     target.tag_cols,
                     target.tag_size,
                     target.tag_spacing};
    aprilgrid_detect(grid, detector, image, cam_K, cam_D);

    // Project AprilGrid
    const auto image_rgb = project_aprilgrid(grid,
                                             camera,
                                             image,
                                             T_WM[i],
                                             T_MC,
                                             T_WF);
    cv::imshow("Validate:", image_rgb);
    cv::waitKey(1);
  }

  // Print newline after print progress has finished
  // print_progress(1.0);
  // std::cout << std::endl;

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
  const calib_config_t config = parse_config(config_file);

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

  // Load calibration data
  std::vector<aprilgrid_t> aprilgrids;
  if (load_camera_calib_data(config.preprocess_path, aprilgrids) != 0) {
    LOG_ERROR("Failed to load camera calibration data!");
    return -1;
  }

  // Load marker poses
  mat4s_t T_WM;
  std::vector<long> timestamps;
  if (load_marker_poses(config, aprilgrids, timestamps, T_WM) != 0) {
    LOG_ERROR("Failed to load marker poses!");
    return -1;
  }

  // Calibrate camera
  LOG_INFO("Calibrating vicon marker!");
  pinhole_t pinhole{config.intrinsics};
  radtan4_t radtan{config.distortion};
  mat4_t T_MC = I(4);
  mat4_t T_WF = I(4);
  retval = calib_vicon_marker_solve(aprilgrids,
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
  std::cout << "T_MC:\n" << T_MC << std::endl;

  // Save results
  const std::string save_path{"./calib_results.yaml"};
  LOG_INFO("Saving optimization results to [%s]", save_path.c_str());
  if (save_results(save_path, config.resolution, pinhole, radtan, T_MC) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", save_path.c_str());
    return -1;
  }

  std::cout << timestamps.size() << std::endl;
  std::cout << T_WM.size() << std::endl;
  validate(config, calib_target, timestamps, T_WM, T_MC, T_WF);

  return 0;
}

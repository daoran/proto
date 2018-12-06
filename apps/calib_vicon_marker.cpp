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
  mat4s_t T_WM;
  int grid_idx = 0;
  std::cout << "grid ts: " << aprilgrids[0].timestamp << std::endl;
  std::cout << "ts: " << timestamps[0] << std::endl;
  for (int i = 0; i < pose_data.rows(); i++) {
    const auto &grid = aprilgrids[grid_idx];
    const long ts = timestamps[i];
    // if (grid.timestamp == ts) {
      // Translation
      const double rx = pose_data(i, 1);
      const double ry = pose_data(i, 2);
      const double rz = pose_data(i, 3);
      const vec3_t r_WM{rx, ry, rz};
      // Rotation
      const double qw = pose_data(i, 4);
      const double qx = pose_data(i, 5);
      const double qy = pose_data(i, 6);
      const double qz = pose_data(i, 7);
      const quat_t q_WM{qw, qx, qy, qz};
      // Add transform T_WM
      T_WM.emplace_back(tf(q_WM, r_WM));
      grid_idx++;
    // }
  }

  std::cout << aprilgrids.size() << std::endl;
  std::cout << T_WM.size() << std::endl;
  exit(0);

  // Calibrate camera
  LOG_INFO("Calibrating camera!");
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

  return 0;
}

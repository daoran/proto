#include <prototype/prototype.hpp>

using namespace prototype;

struct calib_config_t {
  std::string target_file;
  std::string image_path;
  std::string preprocess_path;
  bool imshow = false;

  vec2_t resolution{0.0, 0.0};
  double lens_hfov = 0.0;
  double lens_vfov = 0.0;
  std::string camera_model;
  std::string distortion_model;
};

void print_usage() {
  const std::string usage = R"EOF(
Usage: calib_camera <calib_config.yaml>

The `calib_config.yaml` file is expected to have the following format:

  calib:
    target_file: "aprilgrid_6x6.yaml"
    image_path: "/data/cam0/"
    preprocess_path: "/tmp/calib/mono"

  cam0:
    resolution: [752, 480]
    lens_hfov: 98.0
    lens_vfov: 73.0
    camera_model: "pinhole"
    distortion_model: "radtan"
)EOF";

  std::cout << usage << std::endl;
}

calib_config_t parse_config(const std::string &config_file) {
  config_t config{config_file};
  calib_config_t calib_config;

  parse(config, "calib.target_file", calib_config.target_file);
  parse(config, "calib.image_path", calib_config.image_path);
  parse(config, "calib.preprocess_path", calib_config.preprocess_path);
  parse(config, "calib.imshow", calib_config.imshow, true);


  parse(config, "cam0.resolution", calib_config.resolution);
  parse(config, "cam0.lens_hfov", calib_config.lens_hfov);
  parse(config, "cam0.lens_vfov", calib_config.lens_vfov);
  parse(config, "cam0.camera_model", calib_config.camera_model);
  parse(config, "cam0.distortion_model", calib_config.distortion_model);

  return calib_config;
}

int save_results(const std::string &save_path,
                 const vec2_t &resolution,
                 const pinhole_t &pinhole,
                 const radtan4_t &radtan) {
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

  // Finsh up
  outfile.close();

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
                                      config.resolution,
                                      config.lens_hfov,
                                      config.lens_vfov,
                                      config.preprocess_path,
                                      config.imshow);
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

  // Setup initial camera intrinsics and distortion for optimization
  const double image_width = config.resolution(0);
  const double image_height = config.resolution(1);
  const double fx = pinhole_focal_length(image_width, config.lens_hfov);
  const double fy = pinhole_focal_length(image_height, config.lens_vfov);
  const double cx = config.resolution(0) / 2.0;
  const double cy = config.resolution(1) / 2.0;
  pinhole_t pinhole{fx, fy, cx, cy};
  radtan4_t radtan{0.01, 0.0001, 0.0001, 0.0001};

  // Calibrate camera
  LOG_INFO("Calibrating camera!");
  mat4s_t T_CF;
  if (calib_camera_solve(aprilgrids, pinhole, radtan, T_CF) != 0) {
    LOG_ERROR("Failed to calibrate camera data!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  std::cout << pinhole << std::endl;
  std::cout << radtan << std::endl;

  // Save results
  const std::string save_path{"./calib_results.yaml"};
  LOG_INFO("Saving optimization results to [%s]", save_path.c_str());
  if (save_results(save_path, config.resolution, pinhole, radtan) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", save_path.c_str());
    return -1;
  }

  return 0;
}

#include <prototype/prototype.hpp>

using namespace prototype;

struct calib_config_t {
  std::string target_file;
  std::string image_path;
  std::string preprocess_path;
  bool imshow = true;

  vec2_t resolution{0.0, 0.0};
  std::string camera_model;
  std::string distortion_model;
  vec4_t intrinsics;
  vec4_t distortion;
};

void print_usage() {
  const std::string usage = R"EOF(
Usage: detect_aprilgrid <config.yaml>

The `config.yaml` file is expected to have the following format:

  calib:
    target_file: "aprilgrid_6x6.yaml"
    image_path: "/data/cam0/"
    preprocess_path: "/tmp/calib/mono"

  cam0:
    resolution: [752, 480]
    camera_model: "pinhole"
    distortion_model: "radtan"
    intrinsics: [460.107, 458.985, 371.509, 244.215]
    distortion: [-0.268364, 0.0608506, 0.000938523, -0.000351249]
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
  parse(config, "cam0.camera_model", calib_config.camera_model);
  parse(config, "cam0.distortion_model", calib_config.distortion_model);
  parse(config, "cam0.intrinsics", calib_config.intrinsics);
  parse(config, "cam0.distortion", calib_config.distortion);

  return calib_config;
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
                                      pinhole_K(config.intrinsics),
                                      config.distortion,
                                      config.preprocess_path,
                                      config.imshow);
  if (retval != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  return 0;
}

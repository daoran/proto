#include "prototype/driver/camera/camera.hpp"

namespace prototype {

int camera_config_load(camera_config_t &cc) {
  assert(cc.file_path != "");

  config_t config{cc.file_path};
  if (config.ok == false) {
    LOG_ERROR("Failed to load configure file [%s]!", cc.file_path.c_str());
    return -1;
  }

  // Config variables
  parse(config, "index", cc.index);
  parse(config, "image_width", cc.image_width);
  parse(config, "image_height", cc.image_height);
  parse(config, "image_type", cc.image_type, true);
  cc.ok = true;

  return 0;
}

std::ostream &operator<<(std::ostream &os, const camera_config_t &config) {
  os << "index: " << config.index << std::endl;
  os << "image_width: " << config.image_width << std::endl;
  os << "image_height: " << config.image_height << std::endl;
  return os;
}

int camera_configure(camera_t &cam, const camera_config_t &config) {
  assert(config.ok);
  cam.capture.set(CV_CAP_PROP_FRAME_WIDTH, config.image_width);
  cam.capture.set(CV_CAP_PROP_FRAME_HEIGHT, config.image_height);
  return 0;
}

int camera_connect(camera_t &cam) {
  cam.capture = cv::VideoCapture(cam.camera_index);
  if (cam.capture.isOpened() == 0) {
    LOG_ERROR("Failed to open camera!\n");
    return -1;
  }
  cam.connected = true;
  LOG_INFO("Camera initialized!\n");

  return 0;
}

int camera_disconnect(camera_t &cam) {
  if (cam.connected && cam.capture.isOpened()) {
    cam.capture.release();
    cam.connected = false;
  }

  return 0;
}

int camera_get_frame(camera_t &cam) {
  if (cam.connected == false) {
    return -1;
  }
  cam.capture.read(cam.image);
  cam.last_tic = time_now();

  return 0;
}

} //  namespace prototype

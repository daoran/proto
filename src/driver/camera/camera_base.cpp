#include "prototype/driver/camera/camera_base.hpp"

namespace prototype {

CameraBase::~CameraBase() { this->disconnect(); }

int CameraBase::configure(const std::string &config_path) {
  ConfigParser parser;
  CameraConfig config;
  std::string config_file;
  std::vector<std::string> camera_modes;
  std::vector<std::string> camera_configs;

  // Load config
  config_file = config_path + "/" + "config.yaml";
  parser.addParam("modes", &camera_modes);
  parser.addParam("configs", &camera_configs);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Load camera configs
  for (size_t i = 0; i < camera_modes.size(); i++) {
    config = CameraConfig();
    config_file = config_path + "/" + camera_configs[i];
    if (config.load(config_file) != 0) {
      LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
      return -1;
    }

    this->modes.push_back(camera_modes[i]);
    this->configs[camera_modes[i]] = config;
  }
  this->config = this->configs[camera_modes[0]];
  this->configured = true;

  return 0;
}

int CameraBase::connect() {
  // Pre-check
  if (this->configured == false) {
    return -1;
  }

  // Setup
  const int camera_index = this->config.index;
  const int image_width = this->config.image_width;
  const int image_height = this->config.image_height;

  // Open
  this->capture = new cv::VideoCapture(camera_index);
  if (this->capture->isOpened() == 0) {
    LOG_ERROR("Failed to open camera!\n");
    return -1;

  } else {
    this->capture->set(CV_CAP_PROP_FRAME_WIDTH, image_width);
    this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, image_height);
    this->connected = true;
    LOG_INFO("Camera initialized!\n");
    return 0;
  }

  return 0;
}

int CameraBase::disconnect() {
  if (this->connected && this->capture) {
    this->capture->release();
    this->capture = NULL;
    this->connected = false;
  }

  return 0;
}

int CameraBase::changeMode(const std::string &mode) {
  // Pre-check
  if (this->configs.find(mode) == this->configs.end()) {
    return -1;
  }

  // Update camera settings
  this->config = this->configs[mode];
  this->capture->set(CV_CAP_PROP_FRAME_WIDTH, this->config.image_width);
  this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, this->config.image_height);

  return 0;
}

int CameraBase::roiImage(cv::Mat &image) {
  if (this->config.roi == false) {
    return 0;
  }

  // Crop image to ROI
  const int roi_width = this->config.roi_width;
  const int roi_height = this->config.roi_height;
  const int cx = this->config.camera_matrix.at<double>(0, 2);
  const int cy = this->config.camera_matrix.at<double>(1, 2);
  const Vec2 top_left{cx - roi_width / 2.0, cy - roi_height / 2.0};
  cv::Rect roi(top_left(0), top_left(1), roi_width, roi_height);

  if (roi_width <= 0 || roi_height <= 0) {
    return -1;
  } else {
    image = image(roi);
  }

  return 0;
}

int CameraBase::getFrame(cv::Mat &image) {
  // Pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->connected == false) {
    return -2;
  }

  // Get frame
  this->capture->read(image);

  return 0;
}

int CameraBase::run() {
  // Pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->connected == false) {
    return -2;
  }

  // Setup
  int frame_count = 0;
  double last_tic = time_now();

  // run
  while (true) {
    this->getFrame(this->image);

    // show stats
    this->showFPS(last_tic, frame_count);
    this->showImage(this->image);
  }

  return 0;
}

int CameraBase::showFPS(double &last_tic, int &frame_count) {
  double t;
  double fps;

  // Pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->connected == false) {
    return -2;
  }

  frame_count++;
  if (frame_count % 30 == 0 && this->config.showfps) {
    t = time_now();
    fps = 30.0 / (t - last_tic);
    printf("fps: %.2f\n", fps);
    last_tic = t;
    frame_count = 0;
  }

  return 0;
}

int CameraBase::showImage(cv::Mat &image) {
  // Pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->connected == false) {
    return -2;
  }

  // Show image
  if (this->config.imshow && image.rows && image.cols) {
    cv::imshow("Camera", image);
    cv::waitKey(2);
  }

  return 0;
}

} //  namespace prototype

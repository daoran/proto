#include "calibration/camchain.hpp"

namespace prototype {

Camchain::Camchain() {}

Camchain::~Camchain() {}

int Camchain::load(const int nb_cameras, const std::string &camchain_file) {
  assert(nb_cameras > 0);
  assert(camchain_file.empty() == false);

  // Parse camchain file
  ConfigParser parser;
  CameraProperty cam0, cam1, cam2;
  for (int i = 0; i < nb_cameras; i++) {
    switch (i) {
      case 0:
        parser.addParam("cam0.camera_model", &cam0.camera_model);
        parser.addParam("cam0.distortion_model", &cam0.distortion_model);
        parser.addParam("cam0.distortion_coeffs", &cam0.distortion_coeffs);
        parser.addParam("cam0.intrinsics", &cam0.intrinsics);
        parser.addParam("cam0.resolution", &cam0.resolution);
        break;
      case 1:
        parser.addParam("cam1.camera_model", &cam1.camera_model);
        parser.addParam("cam1.distortion_model", &cam1.distortion_model);
        parser.addParam("cam1.distortion_coeffs", &cam1.distortion_coeffs);
        parser.addParam("cam1.intrinsics", &cam1.intrinsics);
        parser.addParam("cam1.resolution", &cam1.resolution);
        break;
      case 2:
        parser.addParam("cam2.camera_model", &cam2.camera_model);
        parser.addParam("cam2.distortion_model", &cam2.distortion_model);
        parser.addParam("cam2.distortion_coeffs", &cam2.distortion_coeffs);
        parser.addParam("cam2.intrinsics", &cam2.intrinsics);
        parser.addParam("cam2.resolution", &cam2.resolution);
        break;
    }
  }
  parser.addParam("T_C1_C0", &this->T_C1_C0);
  parser.addParam("T_C2_C0.tau_s", &this->tau_s);
  parser.addParam("T_C2_C0.tau_d", &this->tau_d);
  parser.addParam("T_C2_C0.w1", &this->w1);
  parser.addParam("T_C2_C0.w2", &this->w2);
  parser.addParam("T_C2_C0.theta1_offset", &this->theta1_offset);
  parser.addParam("T_C2_C0.theta2_offset", &this->theta2_offset);
  if (parser.load(camchain_file) != 0) {
    return -1;
  }
  this->cam = {cam0, cam1, cam2};

  return 0;
}

int Camchain::save(const std::string &output_path) {
  // Output calibration results
  std::ofstream outfile(output_path);

  const std::string indent = "  ";
  if (outfile.good()) {
    // cam0 calib params
    outfile << "cam0: " << std::endl;
    outfile << indent << "camera_model: " << this->cam[0].camera_model << std::endl;
    outfile << indent << "distortion_model: " << this->cam[0].distortion_model << std::endl;
    outfile << indent << "distortion_coeffs: " << vec2str(this->cam[0].distortion_coeffs) << std::endl;
    outfile << indent << "intrinsics: " << vec2str(this->cam[0].intrinsics) << std::endl;
    outfile << indent << "resolution: " << vec2str(this->cam[0].resolution) << std::endl;
    outfile << std::endl;

    // cam1 calib params
    outfile << "cam1: " << std::endl;
    outfile << indent << "camera_model: " << this->cam[1].camera_model << std::endl;
    outfile << indent << "distortion_model: " << this->cam[1].distortion_model << std::endl;
    outfile << indent << "distortion_coeffs: " << vec2str(this->cam[1].distortion_coeffs) << std::endl;
    outfile << indent << "intrinsics: " << vec2str(this->cam[1].intrinsics) << std::endl;
    outfile << indent << "resolution: " << vec2str(this->cam[1].resolution) << std::endl;
    outfile << std::endl;

    // cam2 calib params
    outfile << "cam2: " << std::endl;
    outfile << indent << "camera_model: " << this->cam[2].camera_model << std::endl;
    outfile << indent << "distortion_model: " << this->cam[2].distortion_model << std::endl;
    outfile << indent << "distortion_coeffs: " << vec2str(this->cam[2].distortion_coeffs) << std::endl;
    outfile << indent << "intrinsics: " << vec2str(this->cam[2].intrinsics) << std::endl;
    outfile << indent << "resolution: " << vec2str(this->cam[2].resolution) << std::endl;
    outfile << std::endl;

    // Transform from cam0 to cam1
    outfile << "T_C1_C0:" << std::endl;
    outfile << indent << "rows: 4" << std::endl;
    outfile << indent << "cols: 4" << std::endl;
    outfile << indent << "data: [" << std::endl;
    for (int i = 0; i < 4; i++) {
      if ((i + 1) != 4) {
        outfile << indent;
        outfile << indent;
        outfile << vec2str(this->T_C1_C0.row(i), false) << "," << std::endl;
      } else {
        outfile << indent;
        outfile << indent;
        outfile << vec2str(this->T_C1_C0.row(i), false) << std::endl;
      }
    }
    outfile << indent << "]" << std::endl;
    outfile << std::endl;

    // Gimbal calib params
    outfile << "T_C2_C0:" << std::endl;
    outfile << indent << "tau_s: [";
    outfile << this->tau_s[0] << ", ";
    outfile << this->tau_s[1] << ", ";
    outfile << this->tau_s[2] << ", ";
    outfile << this->tau_s[3] << ", ";
    outfile << this->tau_s[4] << ", ";
    outfile << this->tau_s[5];
    outfile << "]" << std::endl;

    outfile << indent << "tau_d: [";
    outfile << this->tau_d[0] << ", ";
    outfile << this->tau_d[1] << ", ";
    outfile << this->tau_d[2] << ", ";
    outfile << this->tau_d[3] << ", ";
    outfile << this->tau_d[4] << ", ";
    outfile << this->tau_d[5];
    outfile << "]" << std::endl;

    outfile << indent << "w1: [";
    outfile << this->w1[0] << ", ";
    outfile << this->w1[1] << ", ";
    outfile << this->w1[2];
    outfile << "]" << std::endl;

    outfile << indent << "w2: [";
    outfile << this->w2[0] << ", ";
    outfile << this->w2[1] << ", ";
    outfile << this->w2[2];
    outfile << "]" << std::endl;

    outfile << indent;
    outfile << "theta1_offset: " << this->theta1_offset << std::endl;
    outfile << indent;
    outfile << "theta2_offset: " << this->theta2_offset << std::endl;

  } else {
    return -1;

  }
  outfile.close();

  return 0;
}

std::ostream &operator<<(std::ostream &os, const Camchain &camchain) {
  const std::string indent = "  ";

  // cam0 calib params
  os << "cam0: " << std::endl;
  os << indent << "camera_model: " << camchain.cam[0].camera_model << std::endl;
  os << indent << "distortion_model: " << camchain.cam[0].distortion_model << std::endl;
  os << indent << "distortion_coeffs: " << vec2str(camchain.cam[0].distortion_coeffs) << std::endl;
  os << indent << "intrinsics: " << vec2str(camchain.cam[0].intrinsics) << std::endl;
  os << indent << "resolution: " << vec2str(camchain.cam[0].resolution) << std::endl;
  os << std::endl;

  // cam1 calib params
  os << "cam1: " << std::endl;
  os << indent << "camera_model: " << camchain.cam[1].camera_model << std::endl;
  os << indent << "distortion_model: " << camchain.cam[1].distortion_model << std::endl;
  os << indent << "distortion_coeffs: " << vec2str(camchain.cam[1].distortion_coeffs) << std::endl;
  os << indent << "intrinsics: " << vec2str(camchain.cam[1].intrinsics) << std::endl;
  os << indent << "resolution: " << vec2str(camchain.cam[1].resolution) << std::endl;
  os << std::endl;

  // cam2 calib params
  os << "cam2: " << std::endl;
  os << indent << "camera_model: " << camchain.cam[2].camera_model << std::endl;
  os << indent << "distortion_model: " << camchain.cam[2].distortion_model << std::endl;
  os << indent << "distortion_coeffs: " << vec2str(camchain.cam[2].distortion_coeffs) << std::endl;
  os << indent << "intrinsics: " << vec2str(camchain.cam[2].intrinsics) << std::endl;
  os << indent << "resolution: " << vec2str(camchain.cam[2].resolution) << std::endl;
  os << std::endl;

  // Transform from cam0 to cam1
  os << "T_C1_C0:" << std::endl;
  os << indent << "rows: 4" << std::endl;
  os << indent << "cols: 4" << std::endl;
  os << indent << "data: [" << std::endl;
  for (int i = 0; i < 4; i++) {
    if ((i + 1) != 4) {
      os << indent;
      os << indent;
      os << vec2str(camchain.T_C1_C0.row(i), false) << "," << std::endl;
    } else {
      os << indent;
      os << indent;
      os << vec2str(camchain.T_C1_C0.row(i), false) << std::endl;
    }
  }
  os << indent << "]" << std::endl;
  os << std::endl;

  // Gimbal calib params
  os << "T_C2_C0:" << std::endl;
  os << indent << "tau_s: [";
  os << camchain.tau_s[0] << ", ";
  os << camchain.tau_s[1] << ", ";
  os << camchain.tau_s[2] << ", ";
  os << camchain.tau_s[3] << ", ";
  os << camchain.tau_s[4] << ", ";
  os << camchain.tau_s[5];
  os << "]" << std::endl;

  os << indent << "tau_d: [";
  os << camchain.tau_d[0] << ", ";
  os << camchain.tau_d[1] << ", ";
  os << camchain.tau_d[2] << ", ";
  os << camchain.tau_d[3] << ", ";
  os << camchain.tau_d[4] << ", ";
  os << camchain.tau_d[5];
  os << "]" << std::endl;

  os << indent << "w1: [";
  os << camchain.w1[0] << ", ";
  os << camchain.w1[1] << ", ";
  os << camchain.w1[2];
  os << "]" << std::endl;

  os << indent << "w2: [";
  os << camchain.w2[0] << ", ";
  os << camchain.w2[1] << ", ";
  os << camchain.w2[2];
  os << "]" << std::endl;

  os << indent;
  os << "theta1_offset: " << camchain.theta1_offset << std::endl;
  os << indent;
  os << "theta2_offset: " << camchain.theta2_offset << std::endl;

  return os;
}

} //  namespace prototype

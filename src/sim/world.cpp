#include "sim/world.hpp"

namespace prototype {

SimWorld::SimWorld() {}

SimWorld::~SimWorld() {
  if (this->gnd_file.good()) {
    this->gnd_file.close();
  }

  if (this->mea_file.good()) {
    this->mea_file.close();
  }

  if (this->est_file.good()) {
    this->est_file.close();
  }

  if (this->cam0_idx_file.good()) {
    this->cam0_idx_file.close();
  }
}

int SimWorld::configure(const std::string &config_file) {
  int image_width, image_height;
  double fov;
  matx_t p_points;
  matx_t a_points;
  matx_t joint_setpoints;
  mat4_t T_cam1_cam0 = I(4);
  GimbalModel gimbal_model;

  // Load config file
  ConfigParser parser;
  // -- Simulation settings
  parser.addParam("sim.dt", &this->dt);
  parser.addParam("sim.t_end", &this->t_end);
  parser.addParam("sim.output_path", &this->output_path);
  // -- Camera settings
  parser.addParam("sim.camera.type", &this->camera_type);
  parser.addParam("sim.camera.image_width", &image_width);
  parser.addParam("sim.camera.image_height", &image_height);
  parser.addParam("sim.camera.fov", &fov);
  parser.addParam("sim.camera.T_cam1_cam0", &T_cam1_cam0, true);
  // -- Camera motion settings
  parser.addParam("sim.camera_motion.pos_points", &p_points);
  parser.addParam("sim.camera_motion.att_points", &a_points);
  // -- Gimbal motion settings
  parser.addParam("sim.gimbal_motion.joint_setpoints", &joint_setpoints, true);
  // -- Gimbal settings
  parser.addParam("gimbal.tau_s", &gimbal_model.tau_s, true);
  parser.addParam("gimbal.tau_d", &gimbal_model.tau_d, true);
  parser.addParam("gimbal.w1", &gimbal_model.w1, true);
  parser.addParam("gimbal.w2", &gimbal_model.w2, true);
  parser.addParam("gimbal.theta1_offset", &gimbal_model.theta1_offset, true);
  parser.addParam("gimbal.theta2_offset", &gimbal_model.theta2_offset, true);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Assert
  assert(this->dt > 0.0);
  assert(this->t_end > 0.0);
  assert(this->output_path != "");
  assert(image_width > 0);
  assert(image_height > 0);
  assert(fov > 0);
  assert(p_points.size() > 0);
  assert(a_points.size() > 0);

  // Camera
  const double fx = pinhole_focal_length(image_width, fov);
  const double fy = pinhole_focal_length(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  if (this->camera_type == "MONO_CAMERA") {
    this->mono_camera =
        VirtualCamera(image_width, image_height, fx, fy, cx, cy);
  } else if (this->camera_type == "STATIC_STEREO_CAMERA") {
    if ((T_cam1_cam0 - mat4_t::Identity()).norm() < 1e-6) {
      FATAL("T_cam1_cam0 not configured!");
    }
    this->stereo_camera = VirtualStereoCamera(image_width,
                                              image_height,
                                              fx,
                                              fy,
                                              cx,
                                              cy,
                                              T_cam1_cam0);
  } else if (this->camera_type == "DYNAMIC_STEREO_CAMERA") {
    this->stereo_camera = VirtualStereoCamera(image_width,
                                              image_height,
                                              fx,
                                              fy,
                                              cx,
                                              cy,
                                              gimbal_model);

  } else {
    FATAL("Invalid camera type [%s]\n", this->camera_type.c_str());
  }

  // Camera and gimbal motion
  this->camera_motion = CameraMotion(mat2vec3(p_points),
                                     mat2vec3(a_points),
                                     this->dt,
                                     this->t_end);
  if (this->camera_type == "DYNAMIC_STEREO_CAMERA") {
    this->gimbal_motion = GimbalMotion(mat2vec2(joint_setpoints),
                                       this->dt,
                                       this->t_end);
  }

  // Features
  this->features3d = this->create3DFeaturePerimeter(this->origin,
                                                    this->dimensions,
                                                    this->nb_features);
  this->detectFeatures();
  this->camera_timestamps.push_back(this->t);

  // Output directory and files
  if (this->setupOutput() != 0) {
    LOG_ERROR("Failed to setup simulation output directory / files!");
    return -1;
  }

  // Record initial ground truth, measurement and camera id
  timestamps.push_back(this->t);
  this->recordGroundTruth(this->t,
                          this->camera_motion.p_G,
                          this->camera_motion.v_G,
                          this->camera_motion.rpy_G);
  this->recordMeasurement(this->t,
                          this->camera_motion.a_B,
                          this->camera_motion.w_B);
  if (this->camera_type == "DYNAMIC_STEREO_CAMERA") {
    this->recordJointAngles(this->t, this->gimbal_motion.joint_angles);
  }

  return 0;
}

int SimWorld::setupOutput() {
  // Setup output files
  // -- Make directory if it does not exist already
  if (dir_exists(this->output_path) == false &&
      dir_create(this->output_path) != 0) {
    LOG_ERROR("Failed to create directory [%s] for simulation!",
              output_path.c_str());
    return -1;
  }
  // -- Ground truth file
  this->gnd_file.open(this->output_path + "/ground_truth.csv");
  if (this->gnd_file.good() == false) {
    LOG_ERROR("Failed to open ground truth file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Estimation file
  this->est_file.open(this->output_path + "/estimate.csv");
  if (this->est_file.good() == false) {
    LOG_ERROR("Failed to open estimate file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Camera states file
  this->cam_file.open(this->output_path + "/camera.csv");
  if (this->cam_file.good() == false) {
    LOG_ERROR("Failed to open camera file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Measurement file
  this->mea_file.open(this->output_path + "/measurements.csv");
  if (this->mea_file.good() == false) {
    LOG_ERROR("Failed to open measurement file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Gimbal joints file
  this->jnt_file.open(this->output_path + "/gimbal_joints.csv");
  if (this->jnt_file.good() == false) {
    LOG_ERROR("Failed to open gimbal joints file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Camera 0 file
  dir_create(this->output_path + "/cam0");
  this->cam0_idx_file.open(this->output_path + "/cam0/index.csv");
  if (this->cam0_idx_file.good() == false) {
    LOG_ERROR("Failed to open camera 0 index file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Record 3d features
  mat2csv(this->output_path + "/features.csv", this->features3d);

  // Write header
  // clang-format off
  std::string est_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  est_header += ",x_cov,y_cov,z_cov,vx_cov,vy_cov,vz_cov,roll_cov,pitch_cov,yaw_cov";
  this->est_file << est_header << std::endl;

  const std::string cam_header = "t,x,y,z,roll,pitch,yaw";
  this->cam_file << cam_header << std::endl;

  const std::string mea_header = "t,ax_B,ay_B,az_B,wx_B,wy_B,wz_B";
  this->mea_file << mea_header << std::endl;

  const std::string gnd_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  this->gnd_file << gnd_header << std::endl;

  const std::string cam0_header = "t,frame_id";
  this->cam0_idx_file << cam0_header << std::endl;
  // clang-format on

  return 0;
}

void SimWorld::detectFeaturesWithMonoCamera() {
  // Check what features are observed
  std::vector<int> feature_ids;
  const matx_t kps = this->mono_camera.observedFeatures(this->features3d,
                                                      this->camera_motion.rpy_G,
                                                      this->camera_motion.p_G,
                                                      feature_ids);
  const mat3_t K = this->mono_camera.camera_model.K;

  // Get features lost
  std::vector<int> features_lost;
  std::set_difference(this->features_tracking.begin(),
                      this->features_tracking.end(),
                      feature_ids.begin(),
                      feature_ids.end(),
                      std::inserter(features_lost, features_lost.begin()));

  // Add or updated features observed
  std::vector<size_t> record_feature_ids;
  std::vector<vec2_t> record_keypoints;
  std::vector<vec3_t> record_landmarks;
  for (size_t i = 0; i < feature_ids.size(); i++) {
    const size_t feature_id = feature_ids[i];
    const vec2_t kp = kps.row(i).transpose();
    const vec2_t img_pt = pinhole_pixel2ideal(K, kp);
    const vec3_t ground_truth = this->features3d.row(feature_id).transpose();
    Feature f{img_pt, ground_truth};

    if (this->tracks_tracking.count(feature_id)) {
      // Update feature track
      f.track_id = this->tracks_tracking[feature_id].track_id;
      this->tracks_tracking[feature_id].update(this->frame_index, f);

    } else {
      // Set feature's track id
      f.track_id = this->track_id_counter;

      // Add feature track
      FeatureTrack track;
      track.type = MONO_TRACK;
      track.track_id = this->track_id_counter;
      track.frame_start = this->frame_index;
      track.frame_end = this->frame_index;
      track.track.push_back(f);
      this->tracks_tracking[feature_id] = track;
      this->track_id_counter++;
    }

    // Keep track of keypoints, landmarks and feature for recording
    record_feature_ids.push_back(feature_id);
    record_keypoints.push_back(kp);
    record_landmarks.push_back(ground_truth);
  }
  this->features_tracking = feature_ids;

  // Remove tracks that are too long to lost
  for (auto kv : this->tracks_tracking) {
    const auto feature_id = kv.first;
    const auto track = kv.second;
    if (track.trackedLength() >= this->max_track_length) {
      features_lost.push_back(feature_id);
    }
  }

  // Remove lost features
  for (auto feature_id : features_lost) {
    // Tracks that are too old may already be in feastures_lost
    // hence we need to double check with count
    if (this->tracks_tracking.count(feature_id)) {
      this->tracks_lost.push_back(this->tracks_tracking[feature_id]);
      this->tracks_tracking.erase(this->tracks_tracking.find(feature_id));
    }
  }

  // Record observed keypoints and landmarks
  this->recordCameraObservation(record_feature_ids,
                                record_keypoints,
                                record_landmarks);

  this->frame_index++;
}

void SimWorld::detectFeaturesWithStereoCamera() {
  // Check what features are observed
  std::vector<int> feature_ids;
  const matx_t kps =
      this->stereo_camera.observedFeatures(this->features3d,
                                           this->camera_motion.rpy_G,
                                           this->camera_motion.p_G,
                                           feature_ids);
  const mat3_t K = this->stereo_camera.camera_model.K;

  // Get features lost
  std::vector<int> features_lost;
  std::set_difference(this->features_tracking.begin(),
                      this->features_tracking.end(),
                      feature_ids.begin(),
                      feature_ids.end(),
                      std::inserter(features_lost, features_lost.begin()));

  // Add or updated features observed
  std::vector<size_t> record_feature_ids;
  std::vector<vec2_t> record_keypoints;
  std::vector<vec3_t> record_landmarks;
  for (size_t i = 0; i < feature_ids.size(); i++) {
    const size_t feature_id = feature_ids[i];
    const vec2_t kp0 = kps.row((i * 2)).transpose();
    const vec2_t kp1 = kps.row((i * 2) + 1).transpose();
    const vec2_t p0 = pinhole_pixel2ideal(K, kp0);
    const vec2_t p1 = pinhole_pixel2ideal(K, kp1);
    const vec3_t ground_truth = this->features3d.row(feature_id).transpose();
    Feature f0{p0, ground_truth};
    Feature f1{p1, ground_truth};

    if (this->tracks_tracking.count(feature_id)) {
      // Update feature track
      f0.track_id = this->tracks_tracking[feature_id].track_id;
      f1.track_id = this->tracks_tracking[feature_id].track_id;
      auto &track = this->tracks_tracking[feature_id];

      if (this->stereo_camera.type == "STATIC") {
        track.updateStereo(this->frame_index, f0, f1);
      } else if (this->stereo_camera.type == "DYNAMIC") {
        vec2_t joint_angles = this->stereo_camera.gimbal_model.getJointAngles();
        track.updateStereo(this->frame_index, f0, f1, joint_angles);
      }

    } else {
      // Set feature's track id
      f0.track_id = this->track_id_counter;
      f1.track_id = this->track_id_counter;

      // Add feature track
      FeatureTrack track;
      if (this->stereo_camera.type == "STATIC") {
        track.type = STATIC_STEREO_TRACK;
        track.T_cam1_cam0 = this->stereo_camera.T_cam1_cam0;
      } else if (this->stereo_camera.type == "DYNAMIC") {
        track.type = DYNAMIC_STEREO_TRACK;
        vec2_t joint_angles = this->stereo_camera.gimbal_model.getJointAngles();
        track.joint_angles.push_back(joint_angles);
      }
      track.track_id = this->track_id_counter;
      track.frame_start = this->frame_index;
      track.frame_end = this->frame_index;
      track.track0.push_back(f0);
      track.track1.push_back(f1);
      track.min_track_length = this->min_track_length;

      this->tracks_tracking[feature_id] = track;
      this->track_id_counter++;
    }

    // Keep track of keypoints, landmarks and feature for recording
    record_feature_ids.push_back(feature_id);
    record_keypoints.push_back(kp0);
    record_keypoints.push_back(kp1);
    record_landmarks.push_back(ground_truth);
  }
  this->features_tracking = feature_ids;

  // Remove tracks that are too long to lost
  for (auto kv : this->tracks_tracking) {
    const auto feature_id = kv.first;
    const auto track = kv.second;
    if (track.trackedLength() >= this->max_track_length) {
      features_lost.push_back(feature_id);
    }
  }

  // Remove lost features
  for (auto feature_id : features_lost) {
    // Tracks that are too old may already be in feastures_lost
    // hence we need to double check with count
    if (this->tracks_tracking.count(feature_id)) {
      this->tracks_lost.push_back(this->tracks_tracking[feature_id]);
      this->tracks_tracking.erase(this->tracks_tracking.find(feature_id));
    }
  }

  // Record observed keypoints and landmarks
  this->recordCameraObservation(record_feature_ids,
                                record_keypoints,
                                record_landmarks);

  this->frame_index++;
}

void SimWorld::detectFeatures() {
  if (this->camera_type == "MONO_CAMERA") {
    this->detectFeaturesWithMonoCamera();
  } else if (this->camera_type == "STATIC_STEREO_CAMERA" ||
             this->camera_type == "DYNAMIC_STEREO_CAMERA") {
    this->detectFeaturesWithStereoCamera();
  } else {
    FATAL("Invalid camera type [%s]!", this->camera_type.c_str());
  }
}

FeatureTracks SimWorld::getLostTracks() {
  FeatureTracks lost_tracks;
  for (auto track : this->tracks_lost) {
    if (track.trackedLength() > this->min_track_length) {
      lost_tracks.push_back(track);
    }
  }

  this->tracks_lost.clear();
  return lost_tracks;
}

matx_t SimWorld::create3DFeatures(const struct feature_bounds &bounds,
                                const size_t nb_features) {
  // Create random 3D features
  matx_t features = zeros(nb_features, 3);
  for (size_t i = 0; i < nb_features; i++) {
    features(i, 0) = randf(bounds.x_min, bounds.x_max);
    features(i, 1) = randf(bounds.y_min, bounds.y_max);
    features(i, 2) = randf(bounds.z_min, bounds.z_max);
  }

  return features;
}

matx_t SimWorld::create3DFeaturePerimeter(const vec3_t &origin,
                                        const vec3_t &dimensions,
                                        const size_t nb_features) {
  // Dimension of the outskirt
  const double width = dimensions(0);
  const double length = dimensions(1);
  const double height = dimensions(2);

  // Features per side
  const size_t nb_fps = nb_features / 4.0;

  // Features in the North side
  struct feature_bounds north_bounds;
  north_bounds.x_min = origin(0) - width / 2.0;
  north_bounds.x_max = origin(0) + width / 2.0;
  north_bounds.y_min = origin(1) + length / 2.0;
  north_bounds.y_max = origin(1) + length / 2.0;
  north_bounds.z_min = origin(2) - height / 2.0;
  north_bounds.z_max = origin(2) + height / 2.0;
  const matx_t north_features = this->create3DFeatures(north_bounds, nb_fps);

  // Features in the East side
  struct feature_bounds east_bounds;
  east_bounds.x_min = origin(0) + width / 2.0;
  east_bounds.x_max = origin(0) + width / 2.0;
  east_bounds.y_min = origin(1) - length / 2.0;
  east_bounds.y_max = origin(1) + length / 2.0;
  east_bounds.z_min = origin(2) - height / 2.0;
  east_bounds.z_max = origin(2) + height / 2.0;
  const matx_t east_features = this->create3DFeatures(east_bounds, nb_fps);

  // Features in the South side
  struct feature_bounds south_bounds;
  south_bounds.x_min = origin(0) - width / 2.0;
  south_bounds.x_max = origin(0) + width / 2.0;
  south_bounds.y_min = origin(1) - length / 2.0;
  south_bounds.y_max = origin(1) - length / 2.0;
  south_bounds.z_min = origin(2) - height / 2.0;
  south_bounds.z_max = origin(2) + height / 2.0;
  const matx_t south_features = this->create3DFeatures(south_bounds, nb_fps);

  // Features in the West side
  struct feature_bounds west_bounds;
  west_bounds.x_min = origin(0) - width / 2.0;
  west_bounds.x_max = origin(0) - width / 2.0;
  west_bounds.y_min = origin(1) - length / 2.0;
  west_bounds.y_max = origin(1) + length / 2.0;
  west_bounds.z_min = origin(2) - height / 2.0;
  west_bounds.z_max = origin(2) + height / 2.0;
  const matx_t west_features = this->create3DFeatures(west_bounds, nb_fps);

  // Stack features and return
  matx_t features;
  features = vstack(north_features, east_features);
  features = vstack(features, south_features);
  features = vstack(features, west_features);
  return features;
}

int SimWorld::recordGroundTruth(const double time,
                                const vec3_t &p_G,
                                const vec3_t &v_G,
                                const vec3_t &rpy_G) {
  assert(this->gnd_file.good());

  // -- Time
  this->gnd_file << time << ",";
  // -- Position
  this->gnd_file << p_G(0) << ",";
  this->gnd_file << p_G(1) << ",";
  this->gnd_file << p_G(2) << ",";
  // -- Velocity
  this->gnd_file << v_G(0) << ",";
  this->gnd_file << v_G(1) << ",";
  this->gnd_file << v_G(2) << ",";
  // -- Roll, pitch and yaw
  this->gnd_file << rpy_G(0) << ",";
  this->gnd_file << rpy_G(1) << ",";
  this->gnd_file << rpy_G(2) << std::endl;

  return 0;
}

int SimWorld::recordMeasurement(const double time,
                                const vec3_t &a_B,
                                const vec3_t &w_B) {
  assert(this->mea_file.good());

  // -- Time
  this->mea_file << time << ",";
  // -- Acceleration
  this->mea_file << a_B(0) << ",";
  this->mea_file << a_B(1) << ",";
  this->mea_file << a_B(2) << ",";
  // -- Angular velocity
  this->mea_file << w_B(0) << ",";
  this->mea_file << w_B(1) << ",";
  this->mea_file << w_B(2) << std::endl;

  return 0;
}

int SimWorld::recordCameraObservation(const std::vector<size_t> &feature_ids,
                                      const std::vector<vec2_t> &keypoints,
                                      const std::vector<vec3_t> &landmarks) {
  assert(this->cam0_idx_file.good());

  // Add new entry to camera index
  this->cam0_idx_file << this->t << "," << this->frame_index << std::endl;

  // Setup camera observed features file
  std::string frame_index_str = std::to_string(this->frame_index);
  std::string path = this->output_path + "/cam0/" + frame_index_str + ".csv";
  std::ofstream feature_file(path);
  if (feature_file.good() == false) {
    LOG_ERROR("Failed to open file [%s]!", path.c_str());
    return -1;
  }

  // Output observed features
  if (this->camera_type == "MONO_CAMERA") {
    feature_file << "feature_id,kp_x,kp_y,lm_x,lm_y,lm_z" << std::endl;
    for (size_t i = 0; i < feature_ids.size(); i++) {
      // -- Feature id
      feature_file << feature_ids[i] << ",";
      // -- Keypoint
      feature_file << keypoints[i](0) << ",";
      feature_file << keypoints[i](1) << ",";
      // -- Landmark
      feature_file << landmarks[i](0) << ",";
      feature_file << landmarks[i](1) << ",";
      feature_file << landmarks[i](2) << std::endl;
    }
    feature_file.close();

  } else if (this->camera_type == "STATIC_STEREO_CAMERA" ||
             this->camera_type == "DYNAMIC_STEREO_CAMERA") {
    feature_file << "feature_id,kp0_x,kp0_y,kp1_x,kp1_y,lm_x,lm_y,lm_z"
                 << std::endl;
    for (size_t i = 0; i < feature_ids.size(); i++) {
      // -- Feature id
      feature_file << feature_ids[i] << ",";
      // -- Keypoint
      feature_file << keypoints[(i * 2)](0) << ",";
      feature_file << keypoints[(i * 2)](1) << ",";
      feature_file << keypoints[(i * 2) + 1](0) << ",";
      feature_file << keypoints[(i * 2) + 1](1) << ",";
      // -- Landmark
      feature_file << landmarks[i](0) << ",";
      feature_file << landmarks[i](1) << ",";
      feature_file << landmarks[i](2) << std::endl;
    }
    feature_file.close();

  } else {
    FATAL("Invalid camera type [%s]!", this->camera_type.c_str());
  }

  return 0;
}

int SimWorld::recordEstimate(const double time,
                             const vec3_t &p_G,
                             const vec3_t &v_G,
                             const vec3_t &rpy_G) {
  assert(this->est_file.good());

  // -- Time
  this->est_file << time << ",";
  // -- Position
  this->est_file << p_G(0) << ",";
  this->est_file << p_G(1) << ",";
  this->est_file << p_G(2) << ",";
  // -- Velocity
  this->est_file << v_G(0) << ",";
  this->est_file << v_G(1) << ",";
  this->est_file << v_G(2) << ",";
  // -- Roll, pitch and yaw
  this->est_file << rpy_G(0) << ",";
  this->est_file << rpy_G(1) << ",";
  this->est_file << rpy_G(2) << std::endl;

  return 0;
}

int SimWorld::recordEstimate(const double time,
                             const vec3_t &p_G,
                             const vec3_t &v_G,
                             const vec3_t &rpy_G,
                             const vec3_t &p_G_cov,
                             const vec3_t &v_G_cov,
                             const vec3_t &rpy_G_cov) {
  assert(this->est_file.good());

  // -- Time
  this->est_file << time << ",";
  // -- Position
  this->est_file << p_G(0) << ",";
  this->est_file << p_G(1) << ",";
  this->est_file << p_G(2) << ",";
  // -- Velocity
  this->est_file << v_G(0) << ",";
  this->est_file << v_G(1) << ",";
  this->est_file << v_G(2) << ",";
  // -- Roll, pitch and yaw
  this->est_file << rpy_G(0) << ",";
  this->est_file << rpy_G(1) << ",";
  this->est_file << rpy_G(2) << ",";
  // -- Position covariance
  this->est_file << p_G_cov(0) << ",";
  this->est_file << p_G_cov(1) << ",";
  this->est_file << p_G_cov(2) << ",";
  // -- Velocity covariance
  this->est_file << v_G_cov(0) << ",";
  this->est_file << v_G_cov(1) << ",";
  this->est_file << v_G_cov(2) << ",";
  // -- Roll, pitch and yaw covariance
  this->est_file << rpy_G_cov(0) << ",";
  this->est_file << rpy_G_cov(1) << ",";
  this->est_file << rpy_G_cov(2) << std::endl;

  return 0;
}

int SimWorld::recordCameraStates(const std::vector<double> time,
                                 const std::vector<vec3_t> &p_G,
                                 const std::vector<vec3_t> &rpy_G) {
  assert(this->cam_file.good());

  for (size_t i = 0; i < p_G.size(); i++) {
    // -- Time
    this->cam_file << time[i] << ",";
    // -- Position
    this->cam_file << p_G[i](0) << ",";
    this->cam_file << p_G[i](1) << ",";
    this->cam_file << p_G[i](2) << ",";
    // -- Roll, pitch and yaw
    this->cam_file << rpy_G[i](0) << ",";
    this->cam_file << rpy_G[i](1) << ",";
    this->cam_file << rpy_G[i](2) << std::endl;
  }

  return 0;
}

int SimWorld::recordJointAngles(const double time, const vec2_t &joint_angles) {
  assert(this->jnt_file.good());

  // -- Time
  this->jnt_file << time << ",";
  // -- Position
  this->jnt_file << joint_angles(0) << ",";
  this->jnt_file << joint_angles(1) << std::endl;

  return 0;
}

void SimWorld::generateInitializationData(const int nb_samples,
                                          std::vector<vec3_t> &imu_gyro_buffer,
                                          std::vector<vec3_t> &imu_accel_buffer) {
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> gyro_x_dist{0.0, 0.01};
  std::normal_distribution<> gyro_y_dist{0.0, 0.01};
  std::normal_distribution<> gyro_z_dist{0.0, 0.01};
  std::normal_distribution<> accel_x_dist{0.0, 0.001};
  std::normal_distribution<> accel_y_dist{0.0, 0.001};
  std::normal_distribution<> accel_z_dist{0.0, 0.001};

  for (int i = 0; i < nb_samples; i++) {
    vec3_t w_m{gyro_x_dist(gen), gyro_y_dist(gen), gyro_z_dist(gen)};
    vec3_t a_m{accel_x_dist(gen), accel_y_dist(gen), accel_z_dist(gen) + 9.81};
    imu_gyro_buffer.push_back(w_m);
    imu_accel_buffer.push_back(a_m);
  }
}

int SimWorld::step() {
  // Update robot
  this->t += this->dt;
  this->time_index++;
  this->timestamps.push_back(this->t);

  // Update camera and gimbal motion
  int retval = this->camera_motion.update();
  if (this->camera_type == "DYNAMIC_STEREO_CAMERA") {
    this->gimbal_motion.update();

    const double roll = this->gimbal_motion.joint_angles(0);
    const double pitch = this->gimbal_motion.joint_angles(1);
    this->stereo_camera.setGimbalAttitude(roll, pitch);
  }

  // Observe features
  if ((this->time_index % this->camera_rate) == 0) {
    this->detectFeatures();
    this->camera_timestamps.push_back(this->t);
  }

  // Record ground truth, measurement
  this->recordGroundTruth(this->t,
                          this->camera_motion.p_G,
                          this->camera_motion.v_G,
                          this->camera_motion.rpy_G);
  this->recordMeasurement(this->t,
                          this->camera_motion.a_B,
                          this->camera_motion.w_B);
  if (this->camera_type == "DYNAMIC_STEREO_CAMERA") {
    this->recordJointAngles(this->t, this->gimbal_motion.joint_angles);
  }

  return retval;
}

} //  namespace prototype

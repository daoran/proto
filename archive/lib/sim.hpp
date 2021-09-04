#ifndef PROTO_SIM_HPP
#define PROTO_SIM_HPP

#include "core.hpp"
#include "fs.hpp"
#include "cv.hpp"
#include "data.hpp"

namespace proto {

struct meas_t {
  timestamp_t ts = 0;

  meas_t() {}
  meas_t(const timestamp_t &ts_) : ts{ts_} {}
  virtual ~meas_t() {}
};

struct imu_meas_t {
  timestamp_t ts = 0;
  vec3_t accel{0.0, 0.0, 0.0};
  vec3_t gyro{0.0, 0.0, 0.0};

  imu_meas_t() {}

  imu_meas_t(const timestamp_t &ts_, const vec3_t &accel_, const vec3_t &gyro_)
    : ts{ts_}, accel{accel_}, gyro{gyro_} {}

  ~imu_meas_t() {}
};

struct imu_data_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  timestamps_t timestamps;
  vec3s_t accel;
  vec3s_t gyro;

  void add(const timestamp_t &ts, const vec3_t &acc, const vec3_t &gyr) {
    timestamps.push_back(ts);
    accel.push_back(acc);
    gyro.push_back(gyr);
  }

  size_t size() const { return timestamps.size(); }
  size_t size() { return static_cast<const imu_data_t &>(*this).size(); }

  timestamp_t last_ts() const { return timestamps.back(); }
  timestamp_t last_ts() { return static_cast<const imu_data_t &>(*this).last_ts(); }

  void clear() {
    timestamps.clear();
    accel.clear();
    gyro.clear();
  }
};

struct cam_frame_t {
  timestamp_t ts = 0;
  vec2s_t keypoints;
  std::vector<size_t> feature_ids;

  cam_frame_t() {}

  cam_frame_t(const timestamp_t &ts_,
              const vec2s_t &keypoints_,
              const std::vector<size_t> feature_ids_)
    : ts{ts_}, keypoints{keypoints_}, feature_ids{feature_ids_} {}

  ~cam_frame_t() {}
};

/**
 * SIM IMU
 */
struct sim_imu_t {
  // IMU parameters
  real_t rate = 0.0;        // IMU rate [Hz]
  real_t tau_a = 0.0;       // Reversion time constant for accel [s]
  real_t tau_g = 0.0;       // Reversion time constant for gyro [s]
  real_t sigma_g_c = 0.0;   // Gyro noise density [rad/s/sqrt(Hz)]
  real_t sigma_a_c = 0.0;   // Accel noise density [m/s^s/sqrt(Hz)]
  real_t sigma_gw_c = 0.0;  // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  real_t sigma_aw_c = 0.0;  // Accel drift noise density [m/s^2/sqrt(Hz)]
  real_t g = 9.81;          // Gravity vector [ms-2]

  // IMU flags and biases
  bool started = false;
  vec3_t b_g = zeros(3, 1);
  vec3_t b_a = zeros(3, 1);
  timestamp_t ts_prev = 0;
};

/**
 * Reset IMU
 */
void sim_imu_reset(sim_imu_t &imu);

/**
 * Simulate IMU measurement
 */
void sim_imu_measurement(sim_imu_t &imu,
                         std::default_random_engine &rndeng,
                         const timestamp_t &ts,
                         const mat4_t &T_WS_W,
                         const vec3_t &w_WS_W,
                         const vec3_t &a_WS_W,
                         vec3_t &a_WS_S,
                         vec3_t &w_WS_S);

enum sim_event_type_t {
  NOT_SET,
  CAMERA,
  IMU,
};

struct sim_event_t {
  sim_event_type_t type = NOT_SET;
  int sensor_id = 0;
  timestamp_t ts = 0;
  imu_meas_t imu;
  cam_frame_t frame;

  // Camera event
  sim_event_t(const int sensor_id_,
              const timestamp_t &ts_,
              const vec2s_t &keypoints_,
              const std::vector<size_t> &feature_idxs_)
    : type{CAMERA},
      sensor_id{sensor_id_},
      ts{ts_},
      frame{ts_, keypoints_, feature_idxs_} {}

  // IMU event
  sim_event_t(const int sensor_id_, const timestamp_t &ts_,
              const vec3_t &accel_, const vec3_t &gyro_)
    : type{IMU}, sensor_id{sensor_id_}, ts{ts_}, imu{ts_, accel_, gyro_} {}
};

struct vio_sim_data_t {
  // Settings
  real_t sensor_velocity = 0.3;
  real_t cam_rate = 30;
  real_t imu_rate = 400;

  // Scene data
  vec3s_t features;

  // Camera data
  timestamps_t cam_ts;
  vec3s_t cam_pos_gnd;
  quats_t cam_rot_gnd;
  mat4s_t cam_poses_gnd;
  vec3s_t cam_pos;
  quats_t cam_rot;
  mat4s_t cam_poses;
  std::vector<std::vector<size_t>> observations;
  std::vector<vec2s_t> keypoints;

  // IMU data
  timestamps_t imu_ts;
  vec3s_t imu_acc;
  vec3s_t imu_gyr;
  vec3s_t imu_pos;
  quats_t imu_rot;
  mat4s_t imu_poses;
  vec3s_t imu_vel;

  // Simulation timeline
  std::multimap<timestamp_t, sim_event_t> timeline;

  // Add IMU measurement to timeline
  void add(const int sensor_id,
           const timestamp_t &ts,
           const vec3_t &accel,
           const vec3_t &gyro);

  // Add camera frame to timeline
  void add(const int sensor_id,
           const timestamp_t &ts,
           const vec2s_t &keypoints,
           const std::vector<size_t> &feature_idxs);

  void save(const std::string &dir);
};

void sim_circle_trajectory(const real_t circle_r, vio_sim_data_t &sim_data);

} // namespace proto
#endif // PROTO_SIM_HPP

#include "SimImu.hpp"

namespace cartesian {

SimImu::SimImu() {
  SimCircle sim;

  const Vec3 g{0.0, 0.0, 9.81};
  const double imu_rate = 200.0;

  timestamp_t ts = 0;
  timestamp_t dt_ns = sec2ts(1.0 / imu_rate);
  while (ts <= sec2ts(sim.time_taken)) {
    const double time_s = ts2sec(ts);
    const Mat4 T_WS = sim.get_pose(time_s);
    const Vec3 v_WS = sim.get_velocity(time_s);
    const Vec3 a_S_WS = sim.get_body_acceleration(time_s, g);
    const Vec3 w_S_WS = sim.get_body_angular_velocity(time_s);

    timestamps.push_back(ts);
    poses[ts] = T_WS;
    vel[ts] = v_WS;
    imu_acc[ts] = a_S_WS;
    imu_gyr[ts] = w_S_WS;
    ts += dt_ns;
  }
}

int SimImu::get_num_measurements() const { return timestamps.size(); }

ImuBuffer SimImu::form_imu_buffer(const int start_index,
                                  const int end_index) const {
  ImuBuffer buffer;
  for (int i = 0; i < (end_index - start_index); i++) {
    const timestamp_t ts = timestamps.at(start_index + i);
    const Vec3 acc = imu_acc.at(ts);
    const Vec3 gyr = imu_gyr.at(ts);
    buffer.add(ts, acc, gyr);
  }
  return buffer;
}

void SimImu::save(const std::string &save_path) const {
  // Open csv file
  FILE *csv_file = fopen(save_path.c_str(), "w");

  // Write header
  fprintf(csv_file, "ts,");
  fprintf(csv_file, "x,y,z,");
  fprintf(csv_file, "qx,qy,qz,qw,");
  fprintf(csv_file, "vx,vy,vz,");
  fprintf(csv_file, "ax,ay,az,");
  fprintf(csv_file, "wx,wy,wz\n");

  // Write data
  for (int k = 0; k < get_num_measurements(); k++) {
    const timestamp_t ts = timestamps.at(k);
    const Mat4 pose = poses.at(ts);
    const Vec3 r = tf_trans(pose);
    const Quat q = tf_quat(pose);
    const Vec3 v = vel.at(ts);
    const Vec3 acc = imu_acc.at(ts);
    const Vec3 gyr = imu_gyr.at(ts);

    fprintf(csv_file, "%ld,", ts);
    fprintf(csv_file, "%f,%f,%f,", r.x(), r.y(), r.z());
    fprintf(csv_file, "%f,%f,%f,%f,", q.w(), q.x(), q.y(), q.z());
    fprintf(csv_file, "%f,%f,%f,", v.x(), v.y(), v.z());
    fprintf(csv_file, "%f,%f,%f,", acc.x(), acc.y(), acc.z());
    fprintf(csv_file, "%f,%f,%f\n", gyr.x(), gyr.y(), gyr.z());
  }

  fclose(csv_file);
}

} // namespace cartesian

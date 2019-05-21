#include "prototype/munit.hpp"
#include "prototype/core/file.hpp"
#include "prototype/core/time.hpp"
#include "prototype/core/math.hpp"
#include "prototype/estimation/measurement.hpp"

#define ACCEL0_CSV_PATH "/data/intel_d435i/data/accel0/data.csv"
#define GYRO0_CSV_PATH "/data/intel_d435i/data/gyro0/data.csv"

namespace proto {

void load_accel_data(const std::string &data_path,
                     std::deque<timestamp_t> &accel_ts,
                     std::deque<vec3_t> &accel_data) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double a_x, a_y, a_z = 0.0;
    fscanf(fp, "%" SCNu64 ",%lf,%lf,%lf", &ts, &a_x, &a_y, &a_z);
    accel_ts.push_back(ts);
    accel_data.emplace_back(a_x, a_y, a_z);
  }
  fclose(fp);
}

void load_gyro_data(const std::string &data_path,
                    std::deque<timestamp_t> &gyro_ts,
                    std::deque<vec3_t> &gyro_data) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double w_x, w_y, w_z = 0.0;
    fscanf(fp, "%" SCNu64 ",%lf,%lf,%lf", &ts, &w_x, &w_y, &w_z);
    gyro_ts.push_back(ts);
    gyro_data.emplace_back(w_x, w_y, w_z);
  }
  fclose(fp);
}

int test_interp_timestamps() {
  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel_data;
  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro_data;

  load_gyro_data(GYRO0_CSV_PATH, gyro_ts, gyro_data);
  load_accel_data(ACCEL0_CSV_PATH, accel_ts, accel_data);

  const auto interp_ts = interp_timestamps(gyro_ts, accel_ts);
  MU_CHECK(interp_ts.size() > accel_ts.size());
  MU_CHECK(interp_ts.front() > accel_ts.front());
  MU_CHECK(interp_ts.back() < accel_ts.back());

  return 0;
}

int test_interp_data() {
  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel_data;
  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro_data;

  load_gyro_data(GYRO0_CSV_PATH, gyro_ts, gyro_data);
  load_accel_data(ACCEL0_CSV_PATH, accel_ts, accel_data);
  const auto interp_ts = interp_timestamps(accel_ts, gyro_ts);

  interp_data(interp_ts, accel_ts, accel_data);

  MU_CHECK(interp_ts.size() == accel_ts.size());
  MU_CHECK(interp_ts.size() == accel_data.size());
  MU_CHECK(interp_ts.front() == accel_ts.front());
  MU_CHECK(interp_ts.back() == accel_ts.back());
  for (size_t i = 0; i < accel_ts.size(); i++) {
    MU_CHECK(interp_ts[i] == accel_ts[i]);
  }

  return 0;
}

int test_sync_data() {
  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel_data;
  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro_data;

  load_gyro_data(GYRO0_CSV_PATH, gyro_ts, gyro_data);
  load_accel_data(ACCEL0_CSV_PATH, accel_ts, accel_data);

  const auto interp_ts = interp_timestamps(gyro_ts, accel_ts);
  std::cout << gyro_ts.size() << std::endl;
  std::cout << accel_ts.size() << std::endl;
  sync_data(accel_ts, accel_data, gyro_ts, gyro_data);
  std::cout << gyro_ts.size() << std::endl;
  std::cout << accel_ts.size() << std::endl;

  MU_CHECK(gyro_ts.front() == interp_ts.front());
  MU_CHECK(gyro_ts.back() == interp_ts.back());
  MU_CHECK(accel_ts.front() == interp_ts.front());
  MU_CHECK(accel_ts.back() == interp_ts.back());
  MU_CHECK(gyro_ts.front() == accel_ts.front());
  MU_CHECK(gyro_ts.back() == accel_ts.back());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_interp_timestamps);
  MU_ADD_TEST(test_interp_data);
  MU_ADD_TEST(test_sync_data);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);

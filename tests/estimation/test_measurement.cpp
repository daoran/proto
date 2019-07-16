#include "prototype/munit.hpp"
#include "prototype/core/file.hpp"
#include "prototype/core/time.hpp"
#include "prototype/core/math.hpp"
#include "prototype/dataset/timeline.hpp"
#include "prototype/estimation/measurement.hpp"

#define CAM0_CSV_PATH "/tmp/test_measurement-cam0.csv"
#define ACCEL0_CSV_PATH "/tmp/test_measurement-accel.csv"
#define GYRO0_CSV_PATH "/tmp/test_measurement-gyro.csv"

namespace proto {

void load_camera_data(const std::string &data_path,
                      std::deque<timestamp_t> &camera_ts,
                      const bool skip_header=false) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0 && skip_header) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    fscanf(fp, "%" SCNu64, &ts);
    camera_ts.push_back(ts);
  }
  fclose(fp);
}

void load_accel_data(const std::string &data_path,
                     std::deque<timestamp_t> &accel_ts,
                     std::deque<vec3_t> &accel_data,
                     const bool skip_header=false) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0 && skip_header) {
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
                    std::deque<vec3_t> &gyro_data,
                    const bool skip_header=false) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = file_open(data_path, "r", &nb_rows);
  if (fp == nullptr) {
    FATAL("Failed to open [%s]!", data_path.c_str());
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0 && skip_header) {
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

void save_interpolated_cam0_data(const std::deque<timestamp_t> cam0_ts) {
  std::string out_path = "/tmp/lerp_data-cam0_ts.csv";
  if (ts2csv(out_path, cam0_ts) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }
}

void save_interpolated_gyro_data(const std::deque<timestamp_t> gyro_ts,
                                 const std::deque<vec3_t> gyro_data) {
  std::string out_path = "/tmp/lerp_data-gyro_ts.csv";
  if (ts2csv(out_path, gyro_ts) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }

  out_path = "/tmp/lerp_data-gyro_data.csv";
  if (vec2csv(out_path, gyro_data) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }
}

void save_interpolated_accel_data(const std::deque<timestamp_t> accel_ts,
                                  const std::deque<vec3_t> accel_data) {
  std::string out_path = "/tmp/lerp_data-accel_ts.csv";
  if (ts2csv(out_path, accel_ts) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }

  out_path = "/tmp/lerp_data-accel_data.csv";
  if (vec2csv(out_path, accel_data) != 0) {
    FATAL("Failed to save data to [%s]!\n", out_path.c_str());
  }
}

struct test_data_t {
  std::deque<timestamp_t> cam0_ts;

  std::deque<timestamp_t> accel_ts;
  std::deque<vec3_t> accel_data;

  std::deque<timestamp_t> gyro_ts;
  std::deque<vec3_t> gyro_data;

  test_data_t() {
    OCTAVE_SCRIPT("scripts/measurement/sim_measurements.m");
    load_camera_data(CAM0_CSV_PATH, cam0_ts);
    load_gyro_data(GYRO0_CSV_PATH, gyro_ts, gyro_data);
    load_accel_data(ACCEL0_CSV_PATH, accel_ts, accel_data);
  }
};

int test_lerp_timestamps() {
  test_data_t td;

  const auto lerp_ts = lerp_timestamps(td.gyro_ts, td.accel_ts);
  MU_CHECK(lerp_ts.size() > td.accel_ts.size());
  MU_CHECK(lerp_ts.front() >= td.accel_ts.front());
  MU_CHECK(lerp_ts.back() <= td.accel_ts.back());

  return 0;
}

int test_lerp_data() {
  test_data_t td;
  const auto lerp_ts = lerp_timestamps(td.accel_ts, td.gyro_ts);

  lerp_data(lerp_ts, td.accel_ts, td.accel_data);
  MU_CHECK(lerp_ts.size() == td.accel_ts.size());
  MU_CHECK(lerp_ts.size() == td.accel_data.size());
  MU_CHECK(lerp_ts.front() == td.accel_ts.front());
  MU_CHECK(lerp_ts.back() == td.accel_ts.back());
  for (size_t i = 0; i < td.accel_ts.size(); i++) {
    MU_CHECK(lerp_ts[i] == td.accel_ts[i]);
  }

  // Save interpolated data
  save_interpolated_gyro_data(td.gyro_ts, td.gyro_data);
  save_interpolated_accel_data(td.accel_ts, td.accel_data);

  // Plot data
  // const bool debug = false;
  const bool debug = true;
  if (debug) {
    OCTAVE_SCRIPT("scripts/measurement/plot_lerp.m " \
                  "/tmp/lerp_data-gyro_ts.csv " \
                  "/tmp/lerp_data-gyro_data.csv " \
                  "/tmp/lerp_data-accel_ts.csv " \
                  "/tmp/lerp_data-accel_data.csv " \
                  GYRO0_CSV_PATH " " \
                  ACCEL0_CSV_PATH);
  }

  return 0;
}

int test_lerp_data2() {
  test_data_t td;

  const auto lerp_ts = lerp_timestamps(td.gyro_ts, td.accel_ts);
  lerp_data(td.accel_ts, td.accel_data, td.gyro_ts, td.gyro_data);

  MU_CHECK(td.gyro_ts.front() == lerp_ts.front());
  MU_CHECK(td.gyro_ts.back() == lerp_ts.back());
  MU_CHECK(td.accel_ts.front() == lerp_ts.front());
  MU_CHECK(td.accel_ts.back() == lerp_ts.back());
  MU_CHECK(td.gyro_ts.front() == td.accel_ts.front());
  MU_CHECK(td.gyro_ts.back() == td.accel_ts.back());
  MU_CHECK(td.gyro_ts.size() == td.accel_ts.size());

  // Save interpolated data
  save_interpolated_gyro_data(td.gyro_ts, td.gyro_data);
  save_interpolated_accel_data(td.accel_ts, td.accel_data);

  // Plot data
  const bool debug = false;
  // const bool debug = true;
  if (debug) {
    OCTAVE_SCRIPT("scripts/measurement/plot_lerp.m " \
                  "/tmp/lerp_data-gyro_ts.csv " \
                  "/tmp/lerp_data-gyro_data.csv " \
                  "/tmp/lerp_data-accel_ts.csv " \
                  "/tmp/lerp_data-accel_data.csv " \
                  GYRO0_CSV_PATH " " \
                  ACCEL0_CSV_PATH);
  }

  return 0;
}

int test_vi_data() {
  vi_data_t data;

  MU_CHECK(data.gyro_ts.size() == 0);
  MU_CHECK(data.gyro.size() == 0);

  MU_CHECK(data.accel_ts.size() == 0);
  MU_CHECK(data.accel.size() == 0);

  MU_CHECK(data.camera.size() == 0);

  return 0;
}

int test_vi_data_add_gyro() {
  test_data_t td;

  vi_data_t data;
  for (size_t i = 0; i < td.gyro_ts.size(); i++) {
    vi_data_add_gyro(data, td.gyro_ts.front(), td.gyro_data.front());
    td.gyro_ts.pop_front();
    td.gyro_data.pop_front();
  }

  MU_CHECK(data.gyro_ts.size() > 0);
  MU_CHECK(data.gyro.size() > 0);

  return 0;
}

int test_vi_data_add_accel() {
  test_data_t td;

  vi_data_t data;
  for (size_t i = 0; i < td.accel_ts.size(); i++) {
    vi_data_add_accel(data, td.accel_ts.front(), td.accel_data.front());
    td.accel_ts.pop_front();
    td.accel_data.pop_front();
  }

  MU_CHECK(data.accel_ts.size() > 0);
  MU_CHECK(data.accel.size() > 0);

  return 0;
}

int test_vi_data_lerp() {
  test_data_t td;

  vi_data_t data;
  data.camera[0] = td.cam0_ts;
  data.accel_ts = td.accel_ts;
  data.accel = td.accel_data;
  data.gyro_ts = td.gyro_ts;
  data.gyro = td.gyro_data;

  vi_data_lerp(data);
  save_interpolated_cam0_data(data.camera[0]);
  save_interpolated_gyro_data(data.gyro_ts, data.gyro);
  save_interpolated_accel_data(data.accel_ts, data.accel);

  MU_CHECK(data.accel_ts.size() == data.accel.size());
  MU_CHECK(data.accel_ts.size() == data.gyro_ts.size());
  MU_CHECK(data.accel.size() == data.gyro.size());
  MU_CHECK(data.gyro_ts.size() == data.gyro.size());

  // Make sure there are no repeats in accel timestamps
  {
    auto t_prev = data.accel_ts.front();
    data.accel_ts.pop_front();
    for (const auto ts : data.accel_ts) {
      MU_CHECK(ts > t_prev);
    }
  }

  // Make sure there are no repeats in gyro timestamps
  {
    auto t_prev = data.gyro_ts.front();
    data.gyro_ts.pop_front();
    for (const auto ts : data.gyro_ts) {
      MU_CHECK(ts > t_prev);
    }
  }

  // Plot data
  // const bool debug = false;
  const bool debug = true;
  if (debug) {
    OCTAVE_SCRIPT("scripts/measurement/plot_vi_lerp.m " \
                  "/tmp/lerp_data-cam0_ts.csv " \
                  "/tmp/lerp_data-gyro_ts.csv " \
                  "/tmp/lerp_data-gyro_data.csv " \
                  "/tmp/lerp_data-accel_ts.csv " \
                  "/tmp/lerp_data-accel_data.csv " \
                  CAM0_CSV_PATH " " \
                  GYRO0_CSV_PATH " " \
                  ACCEL0_CSV_PATH);
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_lerp_timestamps);
  MU_ADD_TEST(test_lerp_data);
  // MU_ADD_TEST(test_lerp_data2);
  // MU_ADD_TEST(test_vi_data);
  // MU_ADD_TEST(test_vi_data_add_gyro);
  // MU_ADD_TEST(test_vi_data_add_accel);
  // MU_ADD_TEST(test_vi_data_lerp);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);

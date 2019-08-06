#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

#define OCTAVE_SCRIPT(A)                                                       \
  if (system("octave " A) != 0) {                                              \
    printf("Octave script [%s] failed !", A);                                  \
    exit(-1);                                                                  \
  }

typedef Eigen::Spline<double, 1, 3> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

void save_data(const std::string &save_path,
               const Eigen::RowVectorXd &t,
               const Eigen::RowVectorXd &y) {
  std::ofstream file{save_path};
  if (file.good() != true) {
    printf("Failed to open file for output!");
    exit(-1);
  }

  for (long i = 0; i < t.size(); i++) {
    file << t(i) << "," << y(i) << std::endl;
  }

  file.close();
}

void generate_signal(const size_t size,
                     const double t_end,
                     Eigen::RowVectorXd &time,
                     Eigen::RowVectorXd &signal) {
  // Setup
  time.resize(size + 1);
  signal.resize(size + 1);

  int idx = 0;
  double t = 0;
  double dt = t_end / size;
  double f = 2.0;

  // Generate signal
  while (t <= t_end) {
    // printf("t[%d]: %f\n", idx, t);
    time(idx) = t;
    signal(idx) = sin(2 * M_PI * f * t);
    t += dt;
    idx++;
  }

  // Add last
  time(size) = t_end;
  signal(size) = sin(2 * M_PI * f * t_end);
}

void traverse_spline(const Spline1D &spline,
                     const size_t size,
                     const double t_end,
                     Eigen::RowVectorXd &time,
                     Eigen::RowVectorXd &y) {
  // Setup
  time.resize(size + 1);
  y.resize(size + 1);
  int idx = 0;
  double t = 0.0;
  double dt = t_end / size;

  while (t <= t_end) {
    time(idx) = t;
    y(idx) = spline(t)(0);
    t += dt;
    idx++;
  }

  time(size) = 1.0;
  y(size) = spline(1.0)(0);
}

void traverse_deriv_spline(const Spline1D &spline,
                           const size_t size,
                           const double t_end,
                           Eigen::RowVectorXd &time,
                           Eigen::RowVectorXd &y) {
  // Setup
  time.resize(size + 1);
  y.resize(size + 1);
  int idx = 0;
  double t = 0.0;
  double dt = t_end / size;

  while (t <= t_end) {
    time(idx) = t;
    y(idx) = spline.derivatives(t, 2)(0);
    t += dt;
    idx++;
  }

  time(size) = 1.0;
  y(size) = spline.derivatives(1.0, 2)(0);
}

int main() {
  // Generate and save signal data
  Eigen::RowVectorXd time;
  Eigen::RowVectorXd signal;
  int size = 30;
  double t_end = 1.0;
  generate_signal(size, t_end, time, signal);
  save_data("/tmp/points.csv", time, signal);

  // Fit and generate a spline function
  const auto fit = SplineFitting1D::Interpolate(signal, 3, time);
  Spline1D spline(fit);

  // Traverse spline
  {
    size_t size = 1000;
    Eigen::RowVectorXd t;
    Eigen::RowVectorXd y;
    traverse_spline(spline, size, t_end, t, y);
    save_data("/tmp/spline.csv", t, y);
  }
  {
    size_t size = 1000;
    Eigen::RowVectorXd t;
    Eigen::RowVectorXd y;
    traverse_deriv_spline(spline, size, t_end, t, y);
    save_data("/tmp/spline_deriv.csv", t, y);
  }
  OCTAVE_SCRIPT("scripts/calib/spline.m /tmp/points.csv /tmp/spline.csv /tmp/spline_deriv.csv");

  return 0;
}

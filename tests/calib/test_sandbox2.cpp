#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

#define OCTAVE_SCRIPT(A)                                                       \
  if (system("octave " A) != 0) {                                              \
    printf("Octave script [%s] failed !", A);                                  \
    exit(-1);                                                                  \
  }

#define SPLINE_DEGREE 3
typedef Eigen::Spline<double, 1> Spline1D;
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
  double f = 1.0;

  // Generate signal
  while (t <= t_end) {
    time(idx) = t;
    signal(idx) = sin(2 * M_PI * f * t);
    t += dt;
    idx++;
  }

  // Add last
  time(size) = t_end;
  signal(size) = sin(2 * M_PI * f * t_end);
}

double scale_knot(const double t, const double t_end) {
  return t / t_end;
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
    y(idx) = spline(scale_knot(t, t_end))(0);
    t += dt;
    idx++;
  }

  time(size) = t;
  y(size) = spline(1.0)(0);
}

void traverse_vel_spline(const Spline1D &spline,
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
    const auto p = scale_knot(t, t_end);
    y(idx) = spline.derivatives(p, 1)(1) * 1 / t_end;
    t += dt;
    idx++;
  }

  time(size) = t_end;
  y(size) = spline.derivatives(1.0, 1)(1) * 1 / t_end;
}

void traverse_acc_spline(const Spline1D &spline,
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
    const auto p = scale_knot(t, t_end);
    y(idx) = spline.derivatives(p, 2)(2) * 1 / pow(t_end, 2);
    t += dt;
    idx++;
  }

  time(size) = t_end;
  y(size) = spline.derivatives(1.0, 2)(2) * 1 / pow(t_end, 2);
}

int main() {
  // Generate and save signal data
  Eigen::RowVectorXd time;
  Eigen::RowVectorXd signal;
  int size = 500;
  double t_end = 10.0;
  generate_signal(size, t_end, time, signal);
  save_data("/tmp/points.csv", time, signal);

  // Fit and generate a spline function
  Eigen::RowVectorXd knots;
  knots.resize(time.size());
  for (long i = 0; i < time.size(); i++) {
    knots(i) = scale_knot(time(i), t_end);
  }
  Spline1D spline(SplineFitting1D::Interpolate(signal, SPLINE_DEGREE, knots));

  // Traverse spline
  {
    size_t size = 200;
    Eigen::RowVectorXd t;
    Eigen::RowVectorXd y;
    traverse_spline(spline, size, t_end, t, y);
    save_data("/tmp/spline.csv", t, y);
  }
  {
    size_t size = 200;
    Eigen::RowVectorXd t;
    Eigen::RowVectorXd y;
    traverse_vel_spline(spline, size, t_end, t, y);
    save_data("/tmp/spline_vel.csv", t, y);
  }
  {
    size_t size = 200;
    Eigen::RowVectorXd t;
    Eigen::RowVectorXd y;
    traverse_acc_spline(spline, size, t_end, t, y);
    save_data("/tmp/spline_acc.csv", t, y);
  }
  OCTAVE_SCRIPT("scripts/calib/spline.m "
                "/tmp/points.csv "
                "/tmp/spline.csv "
                "/tmp/spline_vel.csv "
                "/tmp/spline_acc.csv");

  return 0;
}

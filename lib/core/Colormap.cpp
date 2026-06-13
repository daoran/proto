#include "core/Colormap.hpp"

namespace cartesian {
namespace colormap {
namespace internal {

struct Rgb {
  double r;
  double g;
  double b;
};

Vec3i lerp_lut(const Rgb *lut, const int n, const double t) {
  const double t_clamped = std::clamp(t, 0.0, 1.0);
  double x = t_clamped * (n - 1);
  int i = (int) x;
  if (i >= n - 1) {
    i = n - 2;
  }

  double s = x - i;
  return {
      (int) ((lut[i].r + s * (lut[i + 1].r - lut[i].r)) * 255.0),
      (int) ((lut[i].g + s * (lut[i + 1].g - lut[i].g)) * 255.0),
      (int) ((lut[i].b + s * (lut[i + 1].b - lut[i].b)) * 255.0),
  };
}

} // namespace internal

Vec3i jet(const double t) {
  const double t_clamped = std::clamp(t, 0.0, 1.0);
  double r, g, b;

  if (t_clamped < 0.125) {
    double s = t_clamped / 0.125;
    r = 0.0;
    g = 0.0;
    b = 0.5 + 0.5 * s;
  } else if (t_clamped < 0.375) {
    double s = (t_clamped - 0.125) / 0.25;
    r = 0.0;
    g = s;
    b = 1.0;
  } else if (t_clamped < 0.625) {
    double s = (t_clamped - 0.375) / 0.25;
    r = s;
    g = 1.0;
    b = 1.0 - s;
  } else if (t_clamped < 0.875) {
    double s = (t_clamped - 0.625) / 0.25;
    r = 1.0;
    g = 1.0 - s;
    b = 0.0;
  } else {
    double s = (t_clamped - 0.875) / 0.125;
    r = 1.0 - 0.5 * s;
    g = 0.0;
    b = 0.0;
  }
  return {(int) (r * 255.0), (int) (g * 255.0), (int) (b * 255.0)};
}

Vec3i gray(const double t) {
  int v = (int) (std::clamp(t, 0.0, 1.0) * 255.0);
  return {v, v, v};
}

Vec3i hot(const double t) {
  const double t_clamped = std::clamp(t, 0.0, 1.0);
  double r, g, b;
  if (t_clamped < 1.0 / 3.0) {
    double s = t_clamped * 3.0;
    r = s;
    g = 0.0;
    b = 0.0;
  } else if (t_clamped < 2.0 / 3.0) {
    double s = (t_clamped - 1.0 / 3.0) * 3.0;
    r = 1.0;
    g = s;
    b = 0.0;
  } else {
    double s = (t_clamped - 2.0 / 3.0) * 3.0;
    r = 1.0;
    g = 1.0;
    b = s;
  }
  return {(int) (r * 255.0), (int) (g * 255.0), (int) (b * 255.0)};
}

Vec3i cool(const double t) {
  const double t_clamped = std::clamp(t, 0.0, 1.0);
  return {(int) (t_clamped * 255.0), (int) ((1.0 - t_clamped) * 255.0), 255};
}

Vec3i turbo(const double t) {
  static const internal::Rgb lut[] = {
      {0.18995, 0.07176, 0.23217},
      {0.19483, 0.20323, 0.45931},
      {0.21056, 0.33774, 0.58873},
      {0.23942, 0.46894, 0.62050},
      {0.28162, 0.57751, 0.59784},
      {0.33556, 0.66457, 0.53283},
      {0.40215, 0.73058, 0.43352},
      {0.48003, 0.77587, 0.30895},
      {0.56646, 0.80173, 0.18076},
      {0.66075, 0.80344, 0.10515},
      {0.75535, 0.77893, 0.14805},
      {0.83758, 0.72942, 0.21256},
      {0.90482, 0.65745, 0.17317},
      {0.95424, 0.55549, 0.05723},
      {0.98088, 0.42072, 0.01718},
      {0.97174, 0.25304, 0.00752},
      {0.91579, 0.08917, 0.01030},
  };
  return internal::lerp_lut(lut, 17, t);
}

Vec3i viridis(const double t) {
  static const internal::Rgb lut[] = {
      {0.267, 0.004, 0.329},
      {0.275, 0.063, 0.425},
      {0.239, 0.140, 0.474},
      {0.186, 0.218, 0.478},
      {0.133, 0.291, 0.459},
      {0.094, 0.360, 0.426},
      {0.078, 0.426, 0.381},
      {0.114, 0.484, 0.323},
      {0.218, 0.527, 0.247},
      {0.357, 0.548, 0.162},
      {0.503, 0.557, 0.104},
      {0.643, 0.560, 0.119},
      {0.770, 0.557, 0.172},
      {0.880, 0.546, 0.211},
      {0.959, 0.614, 0.207},
      {0.992, 0.770, 0.165},
      {0.993, 0.906, 0.144},
  };
  return lerp_lut(lut, 17, t);
}

Vec3i inferno(const double t) {
  static const internal::Rgb lut[] = {
      {0.001, 0.000, 0.014},
      {0.057, 0.008, 0.268},
      {0.145, 0.022, 0.442},
      {0.262, 0.037, 0.523},
      {0.386, 0.065, 0.519},
      {0.504, 0.115, 0.444},
      {0.610, 0.178, 0.330},
      {0.704, 0.250, 0.207},
      {0.785, 0.328, 0.103},
      {0.853, 0.412, 0.060},
      {0.907, 0.500, 0.082},
      {0.946, 0.594, 0.129},
      {0.969, 0.693, 0.188},
      {0.980, 0.794, 0.266},
      {0.984, 0.889, 0.365},
      {0.988, 0.939, 0.408},
      {0.989, 0.962, 0.422},
  };
  return internal::lerp_lut(lut, 17, t);
}

Vec3i plasma(const double t) {
  static const internal::Rgb lut[] = {
      {0.050, 0.030, 0.530},
      {0.133, 0.020, 0.565},
      {0.228, 0.010, 0.575},
      {0.333, 0.003, 0.555},
      {0.440, 0.007, 0.506},
      {0.545, 0.030, 0.433},
      {0.644, 0.081, 0.341},
      {0.734, 0.152, 0.239},
      {0.813, 0.234, 0.137},
      {0.878, 0.325, 0.057},
      {0.929, 0.421, 0.048},
      {0.964, 0.523, 0.096},
      {0.983, 0.630, 0.163},
      {0.991, 0.739, 0.240},
      {0.994, 0.845, 0.324},
      {0.995, 0.905, 0.376},
      {0.991, 0.816, 0.403},
  };
  return internal::lerp_lut(lut, 17, t);
}

} // namespace colormap
} // namespace cartesian

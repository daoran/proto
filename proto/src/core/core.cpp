#include "proto/core/core.hpp"

namespace proto {

/******************************************************************************
 *                               FILESYSTEM
 *****************************************************************************/

FILE *file_open(const std::string &path,
                const std::string &mode,
                int *nb_rows) {
  FILE *fp = fopen(path.c_str(), mode.c_str());
  if (fp == NULL) {
    return nullptr;
  }

  if (nb_rows != nullptr) {
    *nb_rows = file_rows(path);
  }

  return fp;
}

void skip_line(FILE *fp) {
  char header[BUFSIZ];
  char *retval = fgets(header, BUFSIZ, fp);
  if (retval == NULL) {
    FATAL("Failed to skip line!");
  }
}

int file_rows(const std::string &file_path) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of lines
  int nb_rows = 0;
  std::string line;
  while (std::getline(infile, line)) {
    nb_rows++;
  }

  return nb_rows;
}

int file_copy(const std::string &src, const std::string &dest) {
  // Open input path
  FILE *src_file = fopen(src.c_str(), "rb");
  if (src_file == NULL) {
    fclose(src_file);
    return -1;
  }

  // Open output path
  FILE *dest_file = fopen(dest.c_str(), "wb");
  if (dest_file == NULL) {
    fclose(src_file);
    fclose(dest_file);
    return -2;
  }

  // BUFSIZE default is 8192 bytes
  // BUFSIZE of 1 means one chareter at time
  char buf[BUFSIZ];
  while (size_t size = fread(buf, 1, BUFSIZ, src_file)) {
    fwrite(buf, 1, size, dest_file);
  }

  // Clean up
  fclose(src_file);
  fclose(dest_file);

  return 0;
}

std::string file_ext(const std::string &path) {
  return path.substr(path.find_last_of("."));
}

std::string basename(const std::string &path) {
  auto output = path;
  const size_t last_slash_idx = output.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    output.erase(0, last_slash_idx + 1);
  }

  return output;
}

bool file_exists(const std::string &path) {
  FILE *file;

  file = fopen(path.c_str(), "r");
  if (file != NULL) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

bool dir_exists(const std::string &path) {
  DIR *dir = opendir(path.c_str());

  if (dir) {
    closedir(dir);
    return true;
  } else if (ENOENT == errno) {
    return false;
  } else {
    LOG_ERROR("dir_exists() failed! %s", strerror(errno));
    exit(-1);
  }
}

int dir_create(const std::string &path) {
  std::string command = "mkdir -p " + path;
  return system(command.c_str());
}

std::string dir_name(const std::string &path) {
  std::size_t found = path.find_last_of("/\\");
  return path.substr(0, found);
}

std::string strip(const std::string &s, const std::string &target) {
  size_t first = s.find_first_not_of(target);
  if (std::string::npos == first) {
    return s;
  }

  size_t last = s.find_last_not_of(target);
  return s.substr(first, (last - first + 1));
}

std::string strip_end(const std::string &s, const std::string &target) {
  size_t last = s.find_last_not_of(target);
  return s.substr(0, last + 1);
}

int create_dir(const std::string &path) {
  const std::string command = "mkdir -p " + path;
  const int retval = system(command.c_str());
  if (retval == -1) {
    printf("Error creating directory!n");
    return -1;
  }

  return 0;
}

int remove_dir(const std::string &path) {
  DIR *dir = opendir(path.c_str());
  struct dirent *next_file;

  // pre-check
  if (dir == NULL) {
    return -1;
  }

  // remove files in path
  while ((next_file = readdir(dir)) != NULL) {
    remove(std::string(path + "/" + next_file->d_name).c_str());
  }

  // remove dir
  remove(path.c_str());
  closedir(dir);

  return 0;
}

std::string remove_ext(const std::string &path) {
  auto output = path;
  const size_t period_idx = output.rfind('.');
  if (std::string::npos != period_idx) {
    output.erase(period_idx);
  }
  return output;
}

int list_dir(const std::string &path, std::vector<std::string> &results) {
  struct dirent *entry;
  DIR *dp;

  // Check directory
  dp = opendir(path.c_str());
  if (dp == NULL) {
    return -1;
  }

  // List directory
  while ((entry = readdir(dp))) {
    std::string value(entry->d_name);
    if (value != "." && value != "..") {
      results.push_back(value);
    }
  }

  // Clean up
  closedir(dp);
  return 0;
}

std::vector<std::string> path_split(const std::string path) {
  std::string s;
  std::vector<std::string> splits;

  s = "";
  for (size_t i = 0; i < path.length(); i++) {
    if (s != "" && path[i] == '/') {
      splits.push_back(s);
      s = "";
    } else if (path[i] != '/') {
      s += path[i];
    }
  }
  splits.push_back(s);

  return splits;
}

std::string paths_combine(const std::string path1, const std::string path2) {
  int dirs_up;
  std::string result;
  std::vector<std::string> splits1;
  std::vector<std::string> splits2;

  // setup
  result = "";
  splits1 = path_split(path1);
  splits2 = path_split(path2);

  // obtain number of directory ups in path 2
  dirs_up = 0;
  for (size_t i = 0; i < splits2.size(); i++) {
    if (splits2[i] == "..") {
      dirs_up++;
    }
  }

  // drop path1 elements as path2 dir ups
  for (int i = 0; i < dirs_up; i++) {
    splits1.pop_back();
  }

  // append path1 to result
  if (path1[0] == '/') {
    result += "/";
  }
  for (size_t i = 0; i < splits1.size(); i++) {
    result += splits1[i];
    result += "/";
  }

  // append path2 to result
  for (size_t i = dirs_up; i < splits2.size(); i++) {
    result += splits2[i];
    result += "/";
  }

  // remove trailing slashes
  for (size_t i = result.length() - 1; i > 0; i--) {
    if (result[i] == '/') {
      result.pop_back();
    } else {
      break;
    }
  }

  return result;
}

/******************************************************************************
 *                                ALGEBRA
 *****************************************************************************/

int sign(const double x) {
  if (fltcmp(x, 0.0) == 0) {
    return 0;
  } else if (x < 0) {
    return -1;
  }
  return 1;
}

int fltcmp(const double f1, const double f2) {
  if (fabs(f1 - f2) <= 0.0001) {
    return 0;
  } else if (f1 > f2) {
    return 1;
  } else {
    return -1;
  }
}

double binomial(const double n, const double k) {
  if (k == 0 || k == n) {
    return 1.0;
  }

  return binomial(n - 1, k - 1) + binomial(n - 1, k);
}

/******************************************************************************
 *                            LINEAR ALGEBRA
 *****************************************************************************/

void print_shape(const std::string &name, const matx_t &A) {
  std::cout << name << ": " << A.rows() << "x" << A.cols() << std::endl;
}

void print_shape(const std::string &name, const vecx_t &v) {
  std::cout << name << ": " << v.rows() << "x" << v.cols() << std::endl;
}

void print_array(const std::string &name,
                 const double *array,
                 const size_t size) {
  std::cout << name << std::endl;
  for (size_t i = 0; i < size; i++) {
    printf("%.4f ", array[i]);
  }
  printf("\b\n");
}

void print_vector(const std::string &name, const vecx_t &v) {
  printf("%s: ", name.c_str());
  for (long i = 0; i < v.size(); i++) {
    printf("%f", v(i));
    if ((i + 1) != v.size()) {
      printf(", ");
    }
  }
  printf("\n");
}

void print_matrix(const std::string &name, const matx_t &m) {
  printf("%s:\n", name.c_str());
  for (long i = 0; i < m.rows(); i++) {
    for (long j = 0; j < m.cols(); j++) {
      printf("%f", m(i, j));
      if ((j + 1) != m.cols()) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void print_quaternion(const std::string &name, const quat_t &q) {
  printf("%s: ", name.c_str());
  printf("w:%f, x:%f, y:%f, z:%f\n", q.w(), q.x(), q.y(), q.z());
}

std::string array2str(const double *array, const size_t size) {
  std::stringstream os;
  for (size_t i = 0; i < (size - 1); i++) {
    os << array[i] << " ";
  }
  os << array[size - 1];

  return os.str();
}

void array2vec(const double *x, const size_t size, vecx_t y) {
  y.resize(size);
  for (size_t i = 0; i < size; i++) {
    y(i) = x[i];
  }
}

double *vec2array(const vecx_t &v) {
  double *array = (double *) malloc(sizeof(double) * v.size());
  for (int i = 0; i < v.size(); i++) {
    array[i] = v(i);
  }
  return array;
}

double *mat2array(const matx_t &m) {
  double *array = (double *) malloc(sizeof(double) * m.size());

  int index = 0;
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < m.cols(); j++) {
      array[index] = m(i, j);
      index++;
    }
  }
  return array;
}

double *quat2array(const quat_t &q) {
  double *array = (double *) malloc(sizeof(double) * 4);

  array[0] = q.x();
  array[1] = q.y();
  array[2] = q.z();
  array[3] = q.w();

  return array;
}

void vec2array(const vecx_t &v, double *out) {
  for (int i = 0; i < v.size(); i++) {
    out[i] = v(i);
  }
}

void mat2array(const matx_t &A, double *out) {
  int index = 0;
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      out[index] = A(i, j);
      index++;
    }
  }
}

std::vector<vecx_t> mat2vec(const matx_t &m, bool row_wise) {
  std::vector<vecx_t> vectors;

  if (row_wise) {
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

vec3s_t mat2vec3(const matx_t &m, bool row_wise) {
  vec3s_t vectors;

  if (row_wise) {
    assert(m.cols() == 3);
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    assert(m.rows() == 3);
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

vec2s_t mat2vec2(const matx_t &m, bool row_wise) {
  vec2s_t vectors;

  if (row_wise) {
    assert(m.cols() == 2);
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    assert(m.rows() == 2);
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

matx_t vecs2mat(const vec3s_t &vs) {
  matx_t retval;
  retval.resize(4, vs.size());

  int idx = 0;
  for (const auto &v : vs) {
    const double x = v(0);
    const double y = v(1);
    const double z = v(2);
    retval.block(0, idx, 4, 1) = vec4_t{x, y, z, 1.0};
    idx++;
  }

  return retval;
}

std::string vec2str(const vecx_t &v, bool brackets) {
  std::string str;

  if (brackets) {
    str += "[";
  }

  for (int i = 0; i < v.size(); i++) {
    str += std::to_string(v(i));
    if ((i + 1) != v.size()) {
      str += ", ";
    }
  }

  if (brackets) {
    str += "]";
  }

  return str;
}

std::string arr2str(const double *arr, const size_t len, bool brackets) {
  std::string str;

  if (brackets) {
    str += "[";
  }

  for (size_t i = 0; i < len; i++) {
    str += std::to_string(arr[i]);
    if ((i + 1) != len) {
      str += ", ";
    }
  }

  if (brackets) {
    str += "]";
  }

  return str;
}

std::string mat2str(const matx_t &m, const std::string &indent) {
  std::string str;

  for (int i = 0; i < m.rows(); i++) {
    if ((i + 1) != m.rows()) {
      str += indent;
      str += vec2str(m.row(i), false) + ",\n";
    } else {
      str += indent;
      str += vec2str(m.row(i), false);
    }
  }

  return str;
}

vec3_t normalize(const vec3_t &v) { return v / v.norm(); }

matx_t zeros(const int rows, const int cols) {
  return matx_t::Zero(rows, cols);
}

matx_t zeros(const int size) { return matx_t::Zero(size, size); }

matx_t I(const int rows, const int cols) {
  return matx_t::Identity(rows, cols);
}

matx_t I(const int size) { return matx_t::Identity(size, size); }

matx_t ones(const int rows, const int cols) {
  matx_t A{rows, cols};
  A.fill(1.0);
  return A;
}

matx_t ones(const int size) { return ones(size, size); }

matx_t hstack(const matx_t &A, const matx_t &B) {
  matx_t C(A.rows(), A.cols() + B.cols());
  C << A, B;
  return C;
}

matx_t vstack(const matx_t &A, const matx_t &B) {
  matx_t C(A.rows() + B.rows(), A.cols());
  C << A, B;
  return C;
}

matx_t dstack(const matx_t &A, const matx_t &B) {
  matx_t C = zeros(A.rows() + B.rows(), A.cols() + B.cols());
  C.block(0, 0, A.rows(), A.cols()) = A;
  C.block(A.rows(), A.cols(), B.rows(), B.cols()) = B;
  return C;
}

mat3_t skew(const vec3_t &w) {
  mat3_t S;
  // clang-format off
  S << 0.0, -w(2), w(1),
       w(2), 0.0, -w(0),
       -w(1), w(0), 0.0;
  // clang-format on
  return S;
}

mat3_t skewsq(const vec3_t &w) {
  mat3_t SS = (w * w.transpose()) - pow(w.norm(), 2) * I(3);
  return SS;
}

matx_t enforce_psd(const matx_t &A) {
  matx_t A_psd;

  A_psd.resize(A.rows(), A.cols());

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (i == j) {
        A_psd(i, j) = std::fabs(A(i, j));
      } else {
        const double x = 0.5 * (A(i, j) + A(j, i));
        A_psd(i, j) = x;
        A_psd(j, i) = x;
      }
    }
  }

  return A_psd;
}

matx_t nullspace(const matx_t &A) {
  Eigen::FullPivLU<matx_t> lu(A);
  matx_t A_null_space = lu.kernel();
  return A_null_space;
}

void load_matrix(const std::vector<double> &x,
                 const int rows,
                 const int cols,
                 matx_t &y) {
  int idx;

  // Setup
  idx = 0;
  y.resize(rows, cols);

  // Load matrix
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      y(j, i) = x[idx];
      idx++;
    }
  }
}

void load_matrix(const matx_t &A, std::vector<double> &x) {
  for (int i = 0; i < A.cols(); i++) {
    for (int j = 0; j < A.rows(); j++) {
      x.push_back(A(j, i));
    }
  }
}

/******************************************************************************
 *                                GEOMETRY
 *****************************************************************************/

double sinc(const double x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    static const double c_2 = 1.0 / 6.0;
    static const double c_4 = 1.0 / 120.0;
    static const double c_6 = 1.0 / 5040.0;
    const double x_2 = x * x;
    const double x_4 = x_2 * x_2;
    const double x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

double deg2rad(const double d) { return d * (M_PI / 180.0); }

vec3_t deg2rad(const vec3_t d) { return d * (M_PI / 180.0); }

double rad2deg(const double r) { return r * (180.0 / M_PI); }

vec3_t rad2deg(const vec3_t &r) { return r * (180.0 / M_PI); }

double wrap180(const double euler_angle) {
  return fmod((euler_angle + 180.0), 360.0) - 180.0;
}

double wrap360(const double euler_angle) {
  if (euler_angle > 0) {
    return fmod(euler_angle, 360.0);
  } else {
    return fmod(euler_angle + 360, 360.0);
  }
}

double wrapPi(const double r) { return deg2rad(wrap180(rad2deg(r))); }

double wrap2Pi(const double r) { return deg2rad(wrap360(rad2deg(r))); }

vec2_t circle(const double r, const double theta) {
  return vec2_t{r * cos(theta), r * sin(theta)};
}

vec3_t sphere(const double rho, const double theta, const double phi) {
  const double x = rho * sin(theta) * cos(phi);
  const double y = rho * sin(theta) * sin(phi);
  const double z = rho * cos(theta);
  return vec3_t{x, y, z};
}

mat4_t lookat(const vec3_t &cam_pos,
              const vec3_t &target,
              const vec3_t &up_axis) {
  // Note: If we were using OpenGL the cam_dir would be the opposite direction,
  // since in OpenGL the camera forward is -z. In robotics however our camera
  // is +z forward.
  const vec3_t cam_dir = normalize(target - cam_pos);
  const vec3_t cam_right = normalize(up_axis.cross(cam_dir));
  const vec3_t cam_up = cam_dir.cross(cam_right);

  // clang-format off
  mat4_t A;
  A << cam_right(0), cam_right(1), cam_right(2), 0.0,
       cam_up(0), cam_up(1), cam_up(2), 0.0,
       cam_dir(0), cam_dir(1), cam_dir(2), 0.0,
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // clang-format off
  mat4_t B;
  B << 1.0, 0.0, 0.0, -cam_pos(0),
       0.0, 1.0, 0.0, -cam_pos(1),
       0.0, 0.0, 1.0, -cam_pos(2),
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  mat4_t T_camera_target = A * B;
  mat4_t T_target_camera = T_camera_target.inverse();
  return T_target_camera;
}

double cross_track_error(const vec2_t &p1,
                         const vec2_t &p2,
                         const vec2_t &pos) {
  const double x0 = pos(0);
  const double y0 = pos(1);

  const double x1 = p1(0);
  const double y1 = p1(0);

  const double x2 = p2(0);
  const double y2 = p2(0);

  // calculate perpendicular distance between line (p1, p2) and point (pos)
  const double n = ((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
  const double d = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));

  return fabs(n) / d;
}

int point_left_right(const vec2_t &a, const vec2_t &b, const vec2_t &c) {
  const double a0 = a(0);
  const double a1 = a(1);
  const double b0 = b(0);
  const double b1 = b(1);
  const double c0 = c(0);
  const double c1 = c(1);
  const double x = (b0 - a0) * (c1 - a1) - (b1 - a1) * (c0 - a0);

  if (x > 0) {
    return 1; // left
  } else if (x < 0) {
    return 2; // right
  } else if (x == 0) {
    return 0; // parallel
  }

  return -1;
}

double closest_point(const vec2_t &a,
                     const vec2_t &b,
                     const vec2_t &p,
                     vec2_t &closest) {
  // pre-check
  if ((a - b).norm() == 0) {
    closest = a;
    return -1;
  }

  // calculate closest point
  const vec2_t v1 = p - a;
  const vec2_t v2 = b - a;
  const double t = v1.dot(v2) / v2.squaredNorm();
  closest = a + t * v2;

  return t;
}

void latlon_offset(double lat_ref,
                   double lon_ref,
                   double offset_N,
                   double offset_E,
                   double *lat_new,
                   double *lon_new) {
  *lat_new = lat_ref + (offset_E / EARTH_RADIUS_M);
  *lon_new = lon_ref + (offset_N / EARTH_RADIUS_M) / cos(deg2rad(lat_ref));
}

void latlon_diff(double lat_ref,
                 double lon_ref,
                 double lat,
                 double lon,
                 double *dist_N,
                 double *dist_E) {
  double d_lon = lon - lon_ref;
  double d_lat = lat - lat_ref;

  *dist_N = deg2rad(d_lat) * EARTH_RADIUS_M;
  *dist_E = deg2rad(d_lon) * EARTH_RADIUS_M * cos(deg2rad(lat));
}

double latlon_dist(double lat_ref, double lon_ref, double lat, double lon) {
  double dist_N = 0.0;
  double dist_E = 0.0;

  latlon_diff(lat_ref, lon_ref, lat, lon, &dist_N, &dist_E);
  double dist = sqrt(pow(dist_N, 2) + pow(dist_E, 2));

  return dist;
}

/******************************************************************************
 *                               STATISTICS
 *****************************************************************************/

int randi(int ub, int lb) { return rand() % lb + ub; }

double randf(const double ub, const double lb) {
  const double f = (double) rand() / RAND_MAX;
  return lb + f * (ub - lb);
}

double median(const std::vector<double> &v) {
  // sort values
  std::vector<double> v_copy = v;
  std::sort(v_copy.begin(), v_copy.end());

  // obtain median
  if (v_copy.size() % 2 == 1) {
    // return middle value
    return v_copy[v_copy.size() / 2];

  } else {
    // grab middle two values and calc mean
    const double a = v_copy[v_copy.size() / 2];
    const double b = v_copy[(v_copy.size() / 2) - 1];
    return (a + b) / 2.0;
  }
}

vec3_t mean(const vec3s_t &x) {
  vec3_t x_hat{0.0, 0.0, 0.0};

  for (const auto &v : x) {
    x_hat += v;
  }
  x_hat *= 1.0f / x.size();

  return x_hat;
}

double shannon_entropy(const matx_t &covar) {
  const double n = covar.rows();
  const double covar_det = covar.determinant();
  const double entropy = 0.5 * log(pow(2 * M_PI * exp(1), n) * covar_det);
  return entropy;
}

vec3_t mvn(std::default_random_engine &engine,
           const vec3_t &mu,
           const vec3_t &stdev) {
  std::normal_distribution<double> normal_x(mu(0), stdev(0));
  std::normal_distribution<double> normal_y(mu(1), stdev(1));
  std::normal_distribution<double> normal_z(mu(2), stdev(2));
  return vec3_t{normal_x(engine), normal_y(engine), normal_z(engine)};
}

double gauss_normal() {
  static double V1, V2, S;
  static int phase = 0;
  double X;

  if (phase == 0) {
    do {
      double U1 = (double) rand() / RAND_MAX;
      double U2 = (double) rand() / RAND_MAX;

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    } while (S >= 1 || S == 0);

    X = V1 * sqrt(-2 * log(S) / S);
  } else {
    X = V2 * sqrt(-2 * log(S) / S);
  }

  phase = 1 - phase;
  return X;
}

/*****************************************************************************
 *                               TRANSFORM
 *****************************************************************************/

mat4_t tf(const mat3_t &C, const vec3_t &r) {
  mat4_t T = I(4);
  T.block(0, 0, 3, 3) = C;
  T.block(0, 3, 3, 1) = r;
  return T;
}

mat4_t tf(const quat_t &q, const vec3_t &r) {
  return tf(q.toRotationMatrix(), r);
}

mat4_t tf_perturb_rot(const mat4_t &T, double step_size, const int i) {
  const mat3_t drvec = I(3) * step_size;
  const mat3_t C = tf_rot(T);
  const vec3_t r = tf_trans(T);
  const mat3_t C_diff = rvec2rot(drvec.col(i), 1e-8) * C;
  return tf(C_diff, r);
}

mat4_t tf_perturb_trans(const mat4_t &T, const double step_size, const int i) {
  const mat3_t dr = I(3) * step_size;
  const mat3_t C = tf_rot(T);
  const vec3_t r = tf_trans(T);
  const vec3_t r_diff = r + dr.col(i);
  return tf(C, r_diff);
}

mat3_t rotx(const double theta) {
  mat3_t R;

  // clang-format off
  R << 1.0, 0.0, 0.0,
       0.0, cos(theta), sin(theta),
       0.0, -sin(theta), cos(theta);
  // clang-format on

  return R;
}

mat3_t roty(const double theta) {
  mat3_t R;

  // clang-format off
  R << cos(theta), 0.0, -sin(theta),
       0.0, 1.0, 0.0,
       sin(theta), 0.0, cos(theta);
  // clang-format on

  return R;
}

mat3_t rotz(const double theta) {
  mat3_t R;

  // clang-format off
  R << cos(theta), sin(theta), 0.0,
       -sin(theta), cos(theta), 0.0,
       0.0, 0.0, 1.0;
  // clang-format on

  return R;
}

mat3_t euler123(const vec3_t &euler) {
  // i.e. XYZ rotation sequence (body to world)
  const double phi = euler(0);
  const double theta = euler(1);
  const double psi = euler(2);

  const double R11 = cos(psi) * cos(theta);
  const double R21 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const double R31 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);

  const double R12 = sin(psi) * cos(theta);
  const double R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const double R32 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);

  const double R13 = -sin(theta);
  const double R23 = cos(theta) * sin(phi);
  const double R33 = cos(theta) * cos(phi);

  mat3_t R;
  // clang-format off
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

mat3_t euler321(const vec3_t &euler) {
  // i.e. ZYX rotation sequence (world to body)
  const double phi = euler(0);
  const double theta = euler(1);
  const double psi = euler(2);

  const double R11 = cos(psi) * cos(theta);
  const double R21 = sin(psi) * cos(theta);
  const double R31 = -sin(theta);

  const double R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const double R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const double R32 = cos(theta) * sin(phi);

  const double R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  const double R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  const double R33 = cos(theta) * cos(phi);

  mat3_t R;
  // clang-format off
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

quat_t euler2quat(const vec3_t &euler) {
  const double phi = euler(1);
  const double theta = euler(2);
  const double psi = euler(3);

  const double c_phi = cos(phi / 2.0);
  const double c_theta = cos(theta / 2.0);
  const double c_psi = cos(psi / 2.0);
  const double s_phi = sin(phi / 2.0);
  const double s_theta = sin(theta / 2.0);
  const double s_psi = sin(psi / 2.0);

  const double qx = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
  const double qy = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
  const double qz = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
  const double qw = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;

  const double mag = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  return quat_t{qw / mag, qx / mag, qy / mag, qz / mag};
}

mat3_t vecs2rot(const vec3_t &a_B, const vec3_t &g) {
  // Create Quaternion from two vectors
  const double cos_theta = a_B.normalized().transpose() * g.normalized();
  const double half_cos = sqrt(0.5 * (1.0 + cos_theta));
  const double half_sin = sqrt(0.5 * (1.0 - cos_theta));
  const vec3_t w = a_B.cross(g).normalized();

  const double qw = half_cos;
  const double qx = half_sin * w(0);
  const double qy = half_sin * w(1);
  const double qz = half_sin * w(2);

  // Convert Quaternion to rotation matrix
  const double qx2 = qx * qx;
  const double qy2 = qy * qy;
  const double qz2 = qz * qz;
  const double qw2 = qw * qw;

  const double R11 = qw2 + qx2 - qy2 - qz2;
  const double R12 = 2 * (qx * qy - qw * qz);
  const double R13 = 2 * (qx * qz + qw * qy);

  const double R21 = 2 * (qx * qy + qw * qz);
  const double R22 = qw2 - qx2 + qy2 - qz2;
  const double R23 = 2 * (qy * qz - qw * qx);

  const double R31 = 2 * (qx * qz - qw * qy);
  const double R32 = 2 * (qy * qz + qw * qx);
  const double R33 = qw2 - qx2 - qy2 + qz2;

  mat3_t R;
  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;
  return R;
}

mat3_t rvec2rot(const vec3_t &rvec, const double eps) {
  // Magnitude of rvec
  const double theta = sqrt(rvec.transpose() * rvec);
  // ^ basically norm(rvec), but faster

  // Check if rotation is too small
  if (theta < eps) {
    // clang-format off
    mat3_t R;
    R << 1, -rvec(2), rvec(1),
         rvec(2), 1, -rvec(0),
         -rvec(1), rvec(0), 1;
    return R;
    // clang-format on
  }

  // Convert rvec to rotation matrix
  const vec3_t rvec_normalized = rvec / theta;
  const double x = rvec_normalized(0);
  const double y = rvec_normalized(1);
  const double z = rvec_normalized(2);

  const double c = cos(theta);
  const double s = sin(theta);
  const double C = 1 - c;

  const double xs = x * s;
  const double ys = y * s;
  const double zs = z * s;

  const double xC = x * C;
  const double yC = y * C;
  const double zC = z * C;

  const double xyC = x * yC;
  const double yzC = y * zC;
  const double zxC = z * xC;

  // clang-format off
  mat3_t R;
  R << x * xC + c, xyC - zs, zxC + ys,
       xyC + zs, y * yC + c, yzC - xs,
       zxC - ys, yzC + xs, z * zC + c;
  return R;
  // clang-format on
}

vec3_t quat2euler(const quat_t &q) {
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();

  const double qw2 = qw * qw;
  const double qx2 = qx * qx;
  const double qy2 = qy * qy;
  const double qz2 = qz * qz;

  const double t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const double t2 = asin(2 * (qy * qw - qx * qz));
  const double t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  return vec3_t{t1, t2, t3};
}

void imu_init_attitude(const vec3s_t w_m,
                       const vec3s_t a_m,
                       mat3_t &C_WS,
                       const size_t buffer_size) {
  // Sample IMU measurements
  vec3_t sum_angular_vel = vec3_t::Zero();
  vec3_t sum_linear_acc = vec3_t::Zero();
  for (size_t i = 0; i < buffer_size; i++) {
    sum_angular_vel += w_m[i];
    sum_linear_acc += a_m[i];
  }

  // Initialize the initial orientation, so that the estimation
  // is consistent with the inertial frame.
  const vec3_t mean_accel = sum_linear_acc / buffer_size;
  const vec3_t gravity{0.0, 0.0, -9.81};
  C_WS = vecs2rot(mean_accel, -gravity);
}

/*****************************************************************************
 *                                TIME
 *****************************************************************************/

void timestamp_print(const timestamp_t &ts, const std::string &prefix) {
  printf("%s"
         "%" PRIu64 "\n",
         prefix.c_str(),
         ts);
}

double ts2sec(const timestamp_t &ts) { return ts * 1.0e-9; }

double ns2sec(const uint64_t ns) { return ns * 1.0e-9; }

struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

float toc(struct timespec *tic) {
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

float mtoc(struct timespec *tic) { return toc(tic) * 1000.0; }

double time_now() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}

/*****************************************************************************
 *                                  DATA
 *****************************************************************************/

int8_t int8(const uint8_t *data, const size_t offset) {
  return (int8_t)(data[offset]);
}

uint8_t uint8(const uint8_t *data, const size_t offset) {
  return (uint8_t)(data[offset]);
}

int16_t int16(const uint8_t *data, const size_t offset) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

uint16_t uint16(const uint8_t *data, const size_t offset) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

int32_t sint32(const uint8_t *data, const size_t offset) {
  return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                   (data[offset + 1] << 8) | (data[offset]));
}

uint32_t uint32(const uint8_t *data, const size_t offset) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                    (data[offset + 1] << 8) | (data[offset]));
}

int csvrows(const std::string &file_path) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of lines
  int nb_rows = 0;
  std::string line;
  while (std::getline(infile, line)) {
    nb_rows++;
  }

  return nb_rows;
}

int csvcols(const std::string &file_path) {
  int nb_elements = 1;
  bool found_separator = false;

  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of commas
  std::string line;
  std::getline(infile, line);
  for (size_t i = 0; i < line.length(); i++) {
    if (line[i] == ',') {
      found_separator = true;
      nb_elements++;
    }
  }

  return (found_separator) ? nb_elements : 0;
}

int csv2mat(const std::string &file_path, const bool header, matx_t &data) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of rows and cols
  int nb_rows = csvrows(file_path);
  int nb_cols = csvcols(file_path);

  // Skip header line?
  std::string line;
  if (header) {
    std::getline(infile, line);
    nb_rows -= 1;
  }

  // Load data
  int line_no = 0;
  std::vector<double> vdata;
  data = zeros(nb_rows, nb_cols);

  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    // Load data row
    std::string element;
    for (int i = 0; i < nb_cols; i++) {
      std::getline(ss, element, ',');
      const double value = atof(element.c_str());
      data(line_no, i) = value;
    }

    line_no++;
  }

  return 0;
}

int mat2csv(const std::string &file_path, const matx_t &data) {
  // Open file
  std::ofstream outfile(file_path);
  if (outfile.good() != true) {
    return -1;
  }

  // Save matrix
  for (int i = 0; i < data.rows(); i++) {
    for (int j = 0; j < data.cols(); j++) {
      outfile << data(i, j);

      if ((j + 1) != data.cols()) {
        outfile << ",";
      }
    }
    outfile << "\n";
  }

  // Close file
  outfile.close();
  return 0;
}

int vec2csv(const std::string &file_path, const std::deque<vec3_t> &data) {
  // Open file
  std::ofstream outfile(file_path);
  if (outfile.good() != true) {
    return -1;
  }

  // Save vector
  for (const auto &v : data) {
    outfile << v(0);
    outfile << ",";
    outfile << v(1);
    outfile << ",";
    outfile << v(2);
    outfile << std::endl;
  }

  // Close file
  outfile.close();
  return 0;
}

int ts2csv(const std::string &file_path, const std::deque<timestamp_t> &data) {
  // Open file
  std::ofstream outfile(file_path);
  if (outfile.good() != true) {
    return -1;
  }

  // Save vector
  for (const auto &ts : data) {
    outfile << ts << std::endl;
  }

  // Close file
  outfile.close();
  return 0;
}

void print_progress(const double percentage) {
  const char *PBSTR =
      "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
  const int PBWIDTH = 60;

  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);

  if ((fabs(percentage - 1.0) < 1e-10)) {
    printf("\n");
  }
}

quat_t slerp(const quat_t &q_start, const quat_t &q_end, const double alpha) {
  vec4_t q0{q_start.coeffs().data()};
  vec4_t q1{q_end.coeffs().data()};

  // Only unit quaternions are valid rotations.
  // Normalize to avoid undefined behavior.
  q0.normalize();
  q1.normalize();

  // Compute the cosine of the angle between the two vectors.
  double dot = q0.dot(q1);

  // If the dot product is negative, slerp won't take
  // the shorter path. Note that q1 and -q1 are equivalent when
  // the negation is applied to all four components. Fix by
  // reversing one quaternion.
  if (dot < 0.0f) {
    q1 = -q1;
    dot = -dot;
  }

  const double DOT_THRESHOLD = 0.9995;
  if (dot > DOT_THRESHOLD) {
    // If the inputs are too close for comfort, linearly interpolate
    // and normalize the result.
    vec4_t result = q0 + alpha * (q1 - q0);
    result.normalize();
    return quat_t{result(3), result(0), result(1), result(2)};
  }

  // Since dot is in range [0, DOT_THRESHOLD], acos is safe
  const double theta_0 = acos(dot);     // theta_0 = angle between input vectors
  const double theta = theta_0 * alpha; // theta = angle between q0 and result
  const double sin_theta = sin(theta);  // compute this value only once
  const double sin_theta_0 = sin(theta_0); // compute this value only once

  // == sin(theta_0 - theta) / sin(theta_0)
  const double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
  const double s1 = sin_theta / sin_theta_0;

  const vec4_t result = (s0 * q0) + (s1 * q1);
  return quat_t{result(3), result(0), result(1), result(2)};
}

mat4_t interp_pose(const mat4_t &p0, const mat4_t &p1, const double alpha) {
  // Decompose start pose
  const vec3_t trans0 = tf_trans(p0);
  const quat_t quat0{tf_rot(p0)};

  // Decompose end pose
  const vec3_t trans1 = tf_trans(p1);
  const quat_t quat1{tf_rot(p1)};

  // Interpolate translation and rotation
  const auto trans_interp = lerp(trans0, trans1, alpha);
  const auto quat_interp = quat1.slerp(alpha, quat0);
  // const auto quat_interp = slerp(quat0, quat1, alpha);

  return tf(quat_interp, trans_interp);
}

void interp_poses(const timestamps_t &timestamps,
                  const mat4s_t &poses,
                  const timestamps_t &interp_ts,
                  mat4s_t &interped_poses,
                  const double threshold) {
  assert(timestamps.size() > 0);
  assert(timestamps.size() == poses.size());
  assert(interp_ts.size() > 0);
  assert(timestamps[0] < interp_ts[0]);

  // Interpolation variables
  timestamp_t ts_start = 0;
  timestamp_t ts_end = 0;
  mat4_t pose0 = I(4);
  mat4_t pose1 = I(4);

  size_t interp_idx = 0;
  for (size_t i = 0; i < timestamps.size(); i++) {
    const timestamp_t ts = timestamps[i];
    const mat4_t T = poses[i];

    const double diff = (ts - interp_ts[interp_idx]) * 1e-9;
    if (diff < threshold) {
      // Set interpolation start point
      ts_start = ts;
      pose0 = T;

    } else if (diff > threshold) {
      // Set interpolation end point
      ts_end = ts;
      pose1 = T;

      // Calculate alpha
      const double numerator = (interp_ts[interp_idx] - ts_start) * 1e-9;
      const double denominator = (ts_end - ts_start) * 1e-9;
      const double alpha = numerator / denominator;

      // Interpoate translation and rotation and add to results
      interped_poses.push_back(interp_pose(pose0, pose1, alpha));
      interp_idx++;

      // Shift interpolation current end point to start point
      ts_start = ts_end;
      pose0 = pose1;

      // Reset interpolation end point
      ts_end = 0;
      pose1 = I(4);
    }

    // Check if we're done
    if (interp_idx == interp_ts.size()) {
      break;
    }
  }
}

void closest_poses(const timestamps_t &timestamps,
                   const mat4s_t &poses,
                   const timestamps_t &target_ts,
                   mat4s_t &result) {
  assert(timestamps.size() > 0);
  assert(timestamps.size() == poses.size());
  assert(target_ts.size() > 0);
  assert(timestamps[0] < target_ts[0]);

  // Variables
  const timestamp_t ts = timestamps[0];
  double diff_closest = fabs((ts - target_ts[0]) * 1e-9);
  mat4_t pose_closest = poses[0];

  size_t target_idx = 0;
  for (size_t i = 1; i < timestamps.size(); i++) {
    const timestamp_t ts = timestamps[i];
    const mat4_t pose = poses[i];

    // Find closest pose
    const double diff = fabs((ts - target_ts[target_idx]) * 1e-9);
    if (diff < diff_closest) {
      // Update closest pose
      pose_closest = pose;
      diff_closest = diff;

    } else if (diff > diff_closest) {
      // Add to results
      result.push_back(pose_closest);
      target_idx++;

      // Initialize closest pose with current ts and pose
      diff_closest = fabs((ts - target_ts[target_idx]) * 1e-9);
      pose_closest = pose;
    }

    // Check if we're done
    if (target_idx == target_ts.size()) {
      break;
    }
  }
}

bool all_true(const std::vector<bool> x) {
  for (const auto i : x) {
    if (i == false) {
      return false;
    }
  }

  return true;
}

/******************************************************************************
 *                               CONFIG
 *****************************************************************************/

config_t::config_t() {}

config_t::config_t(const std::string &file_path_) : file_path{file_path_} {
  if (yaml_load_file(file_path_, root) == 0) {
    ok = true;
  }
}

config_t::~config_t() {}

int yaml_load_file(const std::string file_path, YAML::Node &root) {
  // Pre-check
  if (file_exists(file_path) == false) {
    FATAL("File not found: %s", file_path.c_str());
  }

  // Load and parse file
  try {
    root = YAML::LoadFile(file_path);
  } catch (YAML::ParserException &e) {
    LOG_ERROR("%s", e.what());
    return -1;
  }

  return 0;
}

int yaml_get_node(const config_t &config,
                  const std::string &key,
                  const bool optional,
                  YAML::Node &node) {
  assert(config.ok == true);

  // Recurse down config key
  std::vector<YAML::Node> traversal;
  traversal.push_back(config.root);

  std::istringstream iss(key);
  std::string element;

  while (std::getline(iss, element, '.')) {
    traversal.push_back(traversal.back()[element]);
  }
  node = traversal.back();
  // Note:
  //
  //    yaml_node = yaml_node["some_level_deeper"];
  //
  // YAML::Node is mutable, by doing the above it destroys the parsed yaml
  // tree/graph, to avoid this problem we store the visited YAML::Node into
  // a std::vector and return the last visited YAML::Node

  // Check key
  if (node.IsDefined() == false && optional == false) {
    LOG_ERROR("Opps [%s] missing in yaml file [%s]!",
              key.c_str(),
              config.file_path.c_str());
    return -1;
  } else if (node.IsDefined() == false && optional == true) {
    return -1;
  }

  return 0;
}

int yaml_has_key(const config_t &config, const std::string &key) {
  assert(config.ok == true);

  // Recurse down config key
  std::vector<YAML::Node> traversal;
  traversal.push_back(config.root);

  std::istringstream iss(key);
  std::string element;

  while (std::getline(iss, element, '.')) {
    traversal.push_back(traversal.back()[element]);
  }
  auto node = traversal.back();
  // Note:
  //
  //    yaml_node = yaml_node["some_level_deeper"];
  //
  // YAML::Node is mutable, by doing the above it destroys the parsed yaml
  // tree/graph, to avoid this problem we store the visited YAML::Node into
  // a std::vector and return the last visited YAML::Node

  // Check key
  if (node.IsDefined() == false) {
    return -1;
  }

  return 0;
}

int yaml_has_key(const std::string &file_path, const std::string &key) {
  const config_t config{file_path};
  return yaml_has_key(config, key);
}

void yaml_check_matrix_fields(const YAML::Node &node,
                              const std::string &key,
                              size_t &rows,
                              size_t &cols) {
  const std::string targets[3] = {"rows", "cols", "data"};
  for (int i = 0; i < 3; i++) {
    if (!node[targets[i]]) {
      FATAL("Key [%s] is missing for matrix [%s]!",
            targets[i].c_str(),
            key.c_str());
    }
  }
  rows = node["rows"].as<int>();
  cols = node["cols"].as<int>();
}

int parse(const config_t &config,
          const std::string &key,
          vec2_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec2_t>(node, key, optional);
  vec = vec2_t{node[0].as<double>(), node[1].as<double>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec3_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec3_t>(node, key, optional);
  vec =
      vec3_t{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vec4_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_vector<vec4_t>(node, key, optional);
  vec = vec4_t{node[0].as<double>(),
               node[1].as<double>(),
               node[2].as<double>(),
               node[3].as<double>()};
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          vecx_t &vec,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  const size_t vector_size = yaml_check_vector<vecx_t>(node, key, optional);
  vec = vecx_t::Zero(vector_size, 1);
  for (size_t i = 0; i < node.size(); i++) {
    vec(i) = node[i].as<double>();
  }
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          mat2_t &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_matrix<mat2_t>(node, key, optional);
  mat(0, 0) = node["data"][0].as<double>();
  mat(0, 1) = node["data"][1].as<double>();
  mat(1, 0) = node["data"][2].as<double>();
  mat(1, 1) = node["data"][3].as<double>();
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          mat3_t &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_matrix<mat3_t>(node, key, optional);
  // -- Col 1
  mat(0, 0) = node["data"][0].as<double>();
  mat(0, 1) = node["data"][1].as<double>();
  mat(0, 2) = node["data"][2].as<double>();
  // -- Col 2
  mat(1, 0) = node["data"][3].as<double>();
  mat(1, 1) = node["data"][4].as<double>();
  mat(1, 2) = node["data"][5].as<double>();
  // -- Col 3
  mat(2, 0) = node["data"][6].as<double>();
  mat(2, 1) = node["data"][7].as<double>();
  mat(2, 2) = node["data"][8].as<double>();
  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          mat4_t &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  yaml_check_matrix<mat4_t>(node, key, optional);
  size_t index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      mat(i, j) = node["data"][index].as<double>();
      index++;
    }
  }

  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          matx_t &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  size_t rows = 0;
  size_t cols = 0;
  yaml_check_matrix<matx_t>(node, key, optional, rows, cols);

  mat.resize(rows, cols);
  size_t index = 0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      mat(i, j) = node["data"][index].as<double>();
      index++;
    }
  }

  return 0;
}

int parse(const config_t &config,
          const std::string &key,
          cv::Mat &mat,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  size_t rows = 0;
  size_t cols = 0;
  yaml_check_matrix<cv::Mat>(node, key, optional, rows, cols);

  mat = cv::Mat(rows, cols, CV_64F);
  size_t index = 0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      mat.at<double>(i, j) = node["data"][index].as<double>();
      index++;
    }
  }

  return 0;
}

/*****************************************************************************
 *                             INTERPOLATION
 ****************************************************************************/

std::deque<timestamp_t> lerp_timestamps(const std::deque<timestamp_t> &t0,
                                        const std::deque<timestamp_t> &t1) {
  // Determine whether t0 or t1 has a higher rate?
  // Then create interpolation timestamps
  timestamp_t ts_start = 0;
  timestamp_t ts_end = 0;
  std::deque<timestamp_t> base_timestamps;

  if (t0.size() > t1.size()) {
    ts_start = t1.front();
    ts_end = t1.back();
    base_timestamps = t0;
  } else {
    ts_start = t0.front();
    ts_end = t0.back();
    base_timestamps = t1;
  }

  // Form interpolation timestamps
  std::deque<timestamp_t> lerp_ts;
  for (const auto ts : base_timestamps) {
    if (ts >= ts_start && ts <= ts_end) {
      lerp_ts.push_back(ts);
    }
  }

  return lerp_ts;
}

void lerp_data(const std::deque<timestamp_t> &lerp_ts,
               std::deque<timestamp_t> &target_ts,
               std::deque<vec3_t> &target_data,
               const bool keep_old) {
  std::deque<timestamp_t> result_ts;
  std::deque<vec3_t> result_data;

  timestamp_t t0 = target_ts.front();
  timestamp_t t1 = 0;
  vec3_t v0 = target_data.front();
  vec3_t v1 = zeros(3, 1);

  // Loop through target signal
  size_t lerp_idx = 0;
  for (size_t i = 1; i < target_ts.size(); i++) {
    const timestamp_t ts = target_ts[i];
    const vec3_t data = target_data[i];

    // Interpolate
    const bool do_interp = ((ts - lerp_ts[lerp_idx]) * 1e-9) > 0;
    if (do_interp) {
      t1 = ts;
      v1 = data;

      // Loop through interpolation points
      while (lerp_idx < lerp_ts.size()) {
        // Check if interp point is beyond interp end point
        if (t1 < lerp_ts[lerp_idx]) {
          break;
        }

        // Calculate interpolation parameter alpha
        const double num = (lerp_ts[lerp_idx] - t0) * 1e-9;
        const double den = (t1 - t0) * 1e-9;
        const double alpha = num / den;

        // Lerp and add to results
        result_data.push_back(lerp(v0, v1, alpha));
        result_ts.push_back(lerp_ts[lerp_idx]);
        lerp_idx++;
      }

      // Shift interpolation end point to start point
      t0 = t1;
      v0 = v1;

      // Reset interpolation end point
      t1 = 0;
      v1 = zeros(3, 1);
    }

    // Add end point into results, since we are retaining the old data.
    if (keep_old) {
      result_ts.push_back(ts);
      result_data.push_back(data);
    }
  }

  target_ts = result_ts;
  target_data = result_data;
}

static void align_front(const std::deque<timestamp_t> &reference,
                        std::deque<timestamp_t> &target,
                        std::deque<vec3_t> &data) {
  const auto front = reference.front();
  while (true) {
    if (target.front() < front) {
      target.pop_front();
      data.pop_front();
    } else {
      break;
    }
  }
}

static void align_back(const std::deque<timestamp_t> &reference,
                       std::deque<timestamp_t> &target,
                       std::deque<vec3_t> &data) {
  const auto back = reference.back();
  while (true) {
    if (target.back() > back) {
      target.pop_back();
      data.pop_back();
    } else {
      break;
    }
  }
}

void lerp_data(std::deque<timestamp_t> &ts0,
               std::deque<vec3_t> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<vec3_t> &vs1) {
  // Create interpolate timestamps
  auto lerp_ts = lerp_timestamps(ts0, ts1);

  // Interpolate
  if (ts0.size() > ts1.size()) {
    lerp_data(lerp_ts, ts1, vs1);
  } else {
    lerp_data(lerp_ts, ts0, vs0);
  }

  // Chop the front and back so both timestamps and data are sync-ed
  align_front(lerp_ts, ts1, vs1);
  align_front(lerp_ts, ts0, vs0);
  align_back(lerp_ts, ts1, vs1);
  align_back(lerp_ts, ts0, vs0);
}

/*****************************************************************************
 *                                SPLINE
 *****************************************************************************/

ctraj_t::ctraj_t(const timestamps_t &timestamps,
                 const vec3s_t &positions,
                 const quats_t &orientations)
    : timestamps{timestamps}, positions{positions}, orientations{orientations},
      ts_s_start{ts2sec(timestamps.front())},
      ts_s_end{ts2sec(timestamps.back())}, ts_s_gap{ts_s_end - ts_s_start} {
  assert(timestamps.size() == positions.size());
  assert(timestamps.size() == orientations.size());
  assert(timestamps.size() > 4);
  ctraj_init(*this);
}

inline static double ts_normalize(const ctraj_t &ctraj, const timestamp_t ts) {
  const double ts_s_k = ts2sec(ts);
  const double ts_s_start = ctraj.ts_s_start;
  const double ts_s_end = ctraj.ts_s_end;
  return (ts_s_k - ts_s_start) / (ts_s_end - ts_s_start);
}

void ctraj_init(ctraj_t &ctraj) {
  assert(ctraj.timestamps.size() == ctraj.positions.size());
  assert(ctraj.timestamps.size() == ctraj.orientations.size());
  assert(ctraj.timestamps.size() > (3 + 1));

  // Create knots
  const size_t nb_knots = ctraj.timestamps.size();
  Eigen::RowVectorXd knots{nb_knots};
  for (size_t i = 0; i < ctraj.timestamps.size(); i++) {
    knots(i) = ts_normalize(ctraj, ctraj.timestamps[i]);
  }

  // Prep position data
  matx_t pos{3, nb_knots};
  for (size_t i = 0; i < nb_knots; i++) {
    pos.block<3, 1>(0, i) = ctraj.positions[i];
  }

  // Prep orientation data
  matx_t rvec{3, nb_knots};
  Eigen::AngleAxisd aa{ctraj.orientations[0]};
  rvec.block<3, 1>(0, 0) = aa.angle() * aa.axis();

  for (size_t i = 1; i < nb_knots; i++) {
    const Eigen::AngleAxisd aa{ctraj.orientations[i]};
    const vec3_t rvec_k = aa.angle() * aa.axis();
    const vec3_t rvec_km1 = rvec.block<3, 1>(0, i - 1);

    // Calculate delta from rvec_km1 to rvec_k
    vec3_t delta = rvec_k - rvec_km1;
    while (delta.squaredNorm() > (M_PI * M_PI)) {
      delta -= 2 * M_PI * delta.normalized();
    }

    // Add new rotation vector
    rvec.block<3, 1>(0, i) = rvec_km1 + delta;
  }

  // Create splines
  const int spline_degree = 3;
  ctraj.pos_spline = SPLINE3D(pos, knots, spline_degree);
  ctraj.rvec_spline = SPLINE3D(rvec, knots, spline_degree);
}

mat4_t ctraj_get_pose(const ctraj_t &ctraj, const timestamp_t ts) {
  const double u = ts_normalize(ctraj, ts);

  // Translation
  const vec3_t r = ctraj.pos_spline(u);

  // Orientation
  const vec3_t rvec = ctraj.rvec_spline(u);
  if (rvec.norm() < 1e-12) { // Check angle is not zero
    return tf(I(3), r);
  }
  const Eigen::AngleAxisd aa{rvec.norm(), rvec.normalized()};
  const quat_t q{aa};

  return tf(q, r);
}

vec3_t ctraj_get_velocity(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const double u = ts_normalize(ctraj, ts);
  const double scale = (1 / ctraj.ts_s_gap);

  return ctraj.pos_spline.derivatives(u, 1).col(1) * scale;
}

vec3_t ctraj_get_acceleration(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const double u = ts_normalize(ctraj, ts);
  const double scale = pow(1 / ctraj.ts_s_gap, 2);

  return ctraj.pos_spline.derivatives(u, 2).col(2) * scale;
}

static mat3_t so3_exp(const vec3_t &phi) {
  const double norm = phi.norm();
  if (norm < 1e-3) {
    return mat3_t{I(3) + skew(phi)};
  }

  const mat3_t phi_skew = skew(phi);
  mat3_t C = I(3);
  C += (sin(norm) / norm) * phi_skew;
  C += ((1 - cos(norm)) / (norm * norm)) * (phi_skew * phi_skew);

  return C;
}

vec3_t ctraj_get_angular_velocity(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const double u = ts_normalize(ctraj, ts);
  const double scale = 1 / ctraj.ts_s_gap;

  const auto rvec_spline_deriv = ctraj.rvec_spline.derivatives(u, 1);
  const vec3_t rvec = rvec_spline_deriv.col(0);
  const vec3_t rvec_deriv = rvec_spline_deriv.col(1) * scale;

  // Check magnitude of the rotation vector
  const double rvec_norm = rvec.norm();
  if (rvec_norm < 1e-12) {
    return vec3_t{rvec_deriv};
  }

  // Calculate angular velocity
  const mat3_t axis_skew = skew(rvec.normalized());
  vec3_t w =
      (I(3) + axis_skew * (1.0 - cos(rvec_norm)) / rvec_norm +
       axis_skew * axis_skew * (rvec_norm - sin(rvec_norm)) / rvec_norm) *
      rvec_deriv;

  return w;
}

int ctraj_save(const ctraj_t &ctraj, const std::string &save_path) {
  // Setup output file
  std::ofstream file{save_path};
  if (file.good() != true) {
    LOG_ERROR("Failed to open file for output!");
    return -1;
  }

  // Output trajectory timestamps, positions and orientations as csv
  for (size_t i = 0; i < ctraj.timestamps.size(); i++) {
    file << ctraj.timestamps[i] << ",";
    file << ctraj.positions[i](0) << ",";
    file << ctraj.positions[i](1) << ",";
    file << ctraj.positions[i](2) << ",";
    file << ctraj.orientations[i].w() << ",";
    file << ctraj.orientations[i].x() << ",";
    file << ctraj.orientations[i].y() << ",";
    file << ctraj.orientations[i].z() << std::endl;
  }

  // Close file
  file.close();
  return 0;
}

void sim_imu_reset(sim_imu_t &imu) {
  imu.started = false;
  imu.b_g = zeros(3, 1);
  imu.b_a = zeros(3, 1);
  imu.ts_prev = 0;
}

void sim_imu_measurement(sim_imu_t &imu,
                         std::default_random_engine &rndeng,
                         const timestamp_t &ts,
                         const mat4_t &T_WS_W,
                         const vec3_t &w_WS_W,
                         const vec3_t &a_WS_W,
                         vec3_t &a_WS_S,
                         vec3_t &w_WS_S) {
  // Delta time according to sample rate
  double dt = 1.0 / imu.rate;

  // Check consistency of time increments:
  if (imu.started == false) {
    if (fabs(ts2sec(ts - imu.ts_prev) - dt) < (dt / 2.0)) {
      FATAL("Inconsisten sample rate with parameter setting: %f < %f",
            fabs(double(ts - imu.ts_prev) * 1.0e-9 - dt),
            dt / 2.0);
    }
  }

  // IMU initialised?
  if (imu.started == false) {
    // Stationary properties of an Ornstein-Uhlenbeck process
    imu.b_g = mvn(rndeng) * imu.sigma_gw_c * sqrt(imu.tau_g / 2.0);
    imu.b_a = mvn(rndeng) * imu.sigma_aw_c * sqrt(imu.tau_a / 2.0);
    imu.started = true;

  } else {
    // Propagate biases (slow moving signal)
    const vec3_t w_a = mvn(rndeng); // Accel white noise
    imu.b_a += -imu.b_a / imu.tau_a * dt + w_a * imu.sigma_aw_c * sqrt(dt);
    const vec3_t w_g = mvn(rndeng); // Gyro white noise
    imu.b_g += -imu.b_g / imu.tau_g * dt + w_g * imu.sigma_gw_c * sqrt(dt);
  }

  // Compute gyro measurement
  const mat3_t C_SW = tf_rot(T_WS_W).transpose();
  const vec3_t w_g = mvn(rndeng); // Gyro white noise
  w_WS_S = C_SW * w_WS_W + imu.b_g + w_g * imu.sigma_g_c * sqrt(dt);

  // Compute accel measurement
  const vec3_t g{0.0, 0.0, -imu.g}; // Gravity vector
  const vec3_t w_a = mvn(rndeng);   // Accel white noise
  a_WS_S = C_SW * (a_WS_W - g) + imu.b_a + w_a * imu.sigma_a_c * sqrt(dt);
  // TODO: check global gravity direction!

  // Saturate
  // elementwiseSaturate(imu.g_max, w_WS_S);
  // elementwiseSaturate(imu.a_max, a_WS_S);
  imu.ts_prev = ts;
}

/*****************************************************************************
 *                              CONTROL
 *****************************************************************************/

pid_t::pid_t() {}

pid_t::pid_t(const double k_p_, const double k_i_, const double k_d_)
    : k_p{k_p_}, k_i{k_i_}, k_d{k_d_} {}

pid_t::~pid_t() {}

std::ostream &operator<<(std::ostream &os, const pid_t &pid) {
  os << "error_prev:" << pid.error_prev << std::endl;
  os << "error_sum:" << pid.error_sum << std::endl;
  os << "error_p:" << pid.error_p << std::endl;
  os << "error_i:" << pid.error_i << std::endl;
  os << "error_d:" << pid.error_d << std::endl;
  os << "k_p:" << pid.k_p << std::endl;
  os << "k_i:" << pid.k_i << std::endl;
  os << "k_d:" << pid.k_d << std::endl;
  return os;
}

double pid_update(pid_t &p,
                  const double setpoint,
                  const double actual,
                  const double dt) {
  // Calculate errors
  const double error = setpoint - actual;
  p.error_sum += error * dt;

  // Calculate output
  p.error_p = p.k_p * error;
  p.error_i = p.k_i * p.error_sum;
  p.error_d = p.k_d * (error - p.error_prev) / dt;
  const double output = p.error_p + p.error_i + p.error_d;

  p.error_prev = error;
  return output;
}

double pid_update(pid_t &p, const double error, const double dt) {
  return pid_update(p, error, 0.0, dt);
}

void pid_reset(pid_t &p) {
  p.error_prev = 0.0;
  p.error_sum = 0.0;

  // p.error_p = 0.0;
  // p.error_i = 0.0;
  // p.error_d = 0.0;
}

carrot_ctrl_t::carrot_ctrl_t() {}

carrot_ctrl_t::~carrot_ctrl_t() {}

int carrot_ctrl_configure(carrot_ctrl_t &cc,
                          const vec3s_t &waypoints,
                          const double look_ahead_dist) {
  if (waypoints.size() <= (size_t) 2) {
    LOG_ERROR("Too few waypoints!");
    return -1;
  }

  cc.waypoints = waypoints;
  cc.wp_start = cc.waypoints[0];
  cc.wp_end = cc.waypoints[1];
  cc.wp_index = 1;
  cc.look_ahead_dist = look_ahead_dist;

  return 0;
}

int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
                              const vec3_t &pos,
                              vec3_t &result) {
  // Calculate closest point
  const vec3_t v1 = pos - cc.wp_start;
  const vec3_t v2 = cc.wp_end - cc.wp_start;
  const double t = v1.dot(v2) / v2.squaredNorm();
  result = cc.wp_start + t * v2;

  return t;
}

int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
                             const vec3_t &pos,
                             vec3_t &result) {
  vec3_t closest_pt;
  int t = carrot_ctrl_closest_point(cc, pos, closest_pt);

  if (t == -1) {
    // Closest point is before wp_start
    result = cc.wp_start;

  } else if (t == 0) {
    // Closest point is between wp_start wp_end
    const vec3_t u = cc.wp_end - cc.wp_start;
    const vec3_t v = u / u.norm();
    result = closest_pt + cc.look_ahead_dist * v;

  } else if (t == 1) {
    // Closest point is after wp_end
    result = cc.wp_end;
  }

  return t;
}

int carrot_ctrl_update(carrot_ctrl_t &cc,
                       const vec3_t &pos,
                       vec3_t &carrot_pt) {
  // Calculate new carot point
  int status = carrot_ctrl_carrot_point(cc, pos, carrot_pt);

  // Check if there are more waypoints
  if ((cc.wp_index + 1) == cc.waypoints.size()) {
    return 1;
  }

  // Update waypoints
  if (status == 1) {
    cc.wp_index++;
    cc.wp_start = cc.wp_end;
    cc.wp_end = cc.waypoints[cc.wp_index];
  }

  return 0;
}

/*****************************************************************************
 *                             NETWORKING
 *****************************************************************************/

int ip_port_info(const int sockfd, char *ip, int *port) {
  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  if (getpeername(sockfd, (struct sockaddr *) &addr, &len) != 0) {
    return -1;
  }

  // Deal with both IPv4 and IPv6:
  char ipstr[INET6_ADDRSTRLEN];

  if (addr.ss_family == AF_INET) {
    // IPV4
    struct sockaddr_in *s = (struct sockaddr_in *) &addr;
    *port = ntohs(s->sin_port);
    inet_ntop(AF_INET, &s->sin_addr, ipstr, sizeof(ipstr));
  } else {
    // IPV6
    struct sockaddr_in6 *s = (struct sockaddr_in6 *) &addr;
    *port = ntohs(s->sin6_port);
    inet_ntop(AF_INET6, &s->sin6_addr, ipstr, sizeof(ipstr));
  }
  strcpy(ip, ipstr);

  return 0;
}

int ip_port_info(const int sockfd, std::string &ip, int &port) {
  char ipstr[INET6_ADDRSTRLEN];
  const int retval = ip_port_info(sockfd, ipstr, &port);
  ip = std::string{ipstr};
  return retval;
}

tcp_server_t::tcp_server_t(int port_) : port{port_} {}

tcp_client_t::tcp_client_t(const std::string &server_ip_, int server_port_)
    : server_ip{server_ip_}, server_port{server_port_} {}

int tcp_server_config(tcp_server_t &server) {
  // Create socket
  server.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (server.sockfd == -1) {
    LOG_ERROR("Socket creation failed...");
    return -1;
  }

  // Socket options
  const int en = 1;
  const size_t int_sz = sizeof(int);
  if (setsockopt(server.sockfd, SOL_SOCKET, SO_REUSEADDR, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEADDR) failed");
  }
  if (setsockopt(server.sockfd, SOL_SOCKET, SO_REUSEPORT, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEPORT) failed");
  }

  // Assign IP, PORT
  struct sockaddr_in sockaddr;
  bzero(&sockaddr, sizeof(sockaddr));
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  sockaddr.sin_port = htons(server.port);

  // Bind newly created socket to given IP
  int retval =
      bind(server.sockfd, (struct sockaddr *) &sockaddr, sizeof(sockaddr));
  if (retval != 0) {
    LOG_ERROR("Socket bind failed: %s", strerror(errno));
    return -1;
  }

  return 0;
}

int tcp_server_loop(tcp_server_t &server) {
  // Server is ready to listen
  if ((listen(server.sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  }

  // Accept the data packet from client and verification
  std::map<int, pthread_t *> threads;
  int thread_id = 0;

  DEBUG("Server ready!");
  while (true) {
    // Accept incomming connections
    struct sockaddr_in sockaddr;
    socklen_t len = sizeof(sockaddr);
    int connfd = accept(server.sockfd, (struct sockaddr *) &sockaddr, &len);
    if (connfd < 0) {
      LOG_ERROR("Server acccept failed!");
      return -1;
    } else {
      server.conns.push_back(connfd);
    }

    // Fork off a thread to handle the connection
    pthread_t thread;
    pthread_create(&thread, nullptr, server.conn_thread, (void *) &server);
    threads.insert({thread_id, &thread});
    thread_id++;
  }
  DEBUG("Server shutting down ...");

  return 0;
}

int tcp_client_config(tcp_client_t &client) {
  // Create socket
  client.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (client.sockfd == -1) {
    LOG_ERROR("Socket creation failed!");
    return -1;
  }

  // Assign IP, PORT
  struct sockaddr_in server;
  size_t server_size = sizeof(server);
  bzero(&server, server_size);
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(client.server_ip.c_str());
  server.sin_port = htons(client.server_port);

  // Connect to server
  if (connect(client.sockfd, (struct sockaddr *) &server, server_size) != 0) {
    LOG_ERROR("Failed to connect to server!");
    return -1;
  }
  DEBUG("Connected to the server!");

  return 0;
}

int tcp_client_loop(tcp_client_t &client) {
  while (true) {
    if (client.loop_cb) {
      int retval = client.loop_cb(client);
      switch (retval) {
      case -1: return -1;
      case 1: break;
      }
    }
  }

  return 0;
}

} // namespace proto

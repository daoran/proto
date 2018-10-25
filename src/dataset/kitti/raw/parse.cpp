#include "prototype/dataset/kitti/raw/parse.hpp"

namespace prototype {

std::string parse_string(const std::string &line) {
  return strip(line.substr(line.find(":") + 1, line.size() - 1));
}

double parse_double(const std::string &line) {
  const std::string str_val = parse_string(line);
  const double val = atof(str_val.c_str());
  return val;
}

std::vector<double> parse_array(const std::string &line) {
  // Setup
  std::string s = parse_string(line);
  std::string delimiter = " ";

  // Extract tokens
  size_t pos = 0;
  std::string token;
  std::vector<double> values;

  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    values.push_back(std::stod(token));
    s.erase(0, pos + delimiter.length());
  }

  // Extract last token
  token = s.substr(0, pos);
  values.push_back(std::stod(token));

  return values;
}

vec2_t parse_vec2(const std::string &line) {
  const std::vector<double> values = parse_array(line);
  return vec2_t(values[0], values[1]);
}

vec3_t parse_vec3(const std::string &line) {
  const std::vector<double> values = parse_array(line);
  return vec3_t(values[0], values[1], values[2]);
}

vecx_t parse_vecx(const std::string &line) {
  const std::vector<double> values = parse_array(line);

  // Form the vector
  vecx_t vec;
  vec.resize(values.size());
  for (size_t i = 0; i < values.size(); i++) {
    vec(i) = values[i];
  }

  return vec;
}

mat3_t parse_mat3(const std::string &line) {
  const std::vector<double> values = parse_array(line);

  // Form the matrix
  mat3_t mat;
  int index = 0;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      mat(i, j) = values[index];
      index++;
    }
  }

  return mat;
}

mat34_t parse_mat34(const std::string &line) {
  const std::vector<double> values = parse_array(line);

  // Form the matrix
  mat34_t mat;
  int index = 0;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 4; j++) {
      mat(i, j) = values[index];
      index++;
    }
  }

  return mat;
}

int parse_timestamp(const std::string &line, long *s) {
  // Parse datetime string
  unsigned int year = 0;
  unsigned int month = 0;
  unsigned int day = 0;
  unsigned int hour = 0;
  unsigned int minute = 0;
  unsigned int second = 0;
  unsigned int subsecond = 0;

  int scanned = std::sscanf(line.c_str(),
                            "%4u-%2u-%2u %2u:%2u:%2u.%u",
                            &year,
                            &month,
                            &day,
                            &hour,
                            &minute,
                            &second,
                            &subsecond);
  if (scanned != 7) {
    return -1;
  }

  // Convert datetime to milliseconds since epoch (1970)
  struct tm t = {}; // IMPORTANT: MUST INITIALIZE TO ZERO ELSE BUGS...
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  *s = ((double) mktime(&t) * 1.0e9) + subsecond;

  return 0;
}

} //  namespace prototype

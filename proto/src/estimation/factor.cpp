#include "proto/estimation/factor.hpp"

namespace proto {

void pose_print(const std::string &prefix, const pose_t &pose) {
  const quat_t q = pose.rot();
  const vec3_t r = pose.trans();

  printf("[%s] ", prefix.c_str());
  printf("q: (%f, %f, %f, %f)", q.w(), q.x(), q.y(), q.z());
  printf("\t");
  printf("r: (%f, %f, %f)\n", r(0), r(1), r(2));
}

poses_t load_poses(const std::string &csv_path) {
  FILE *csv_file = fopen(csv_path.c_str(), "r");
  char line[1024] = {0};
  poses_t poses;

  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[1024] = {0};
    real_t data[7] = {0};
    int index = 0;
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[index] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        index++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    quat_t q{data[0], data[1], data[2], data[3]};
    vec3_t r{data[4], data[5], data[6]};
    poses.emplace_back(data);
  }
  fclose(csv_file);

  return poses;
}

static keypoints_t parse_keypoints_line(const char *line) {
  char entry[100] = {0};
  int kp_ready = 0;
  vec2_t kp{0.0, 0.0};
  int kp_index = 0;
  bool first_element_parsed = false;

  // Parse line
  keypoints_t keypoints;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (first_element_parsed == false) {
        first_element_parsed = true;
      } else {
        // Parse keypoint
        if (kp_ready == 0) {
          kp(0) = strtod(entry, NULL);
          kp_ready = 1;

        } else {
          kp(1) = strtod(entry, NULL);
          keypoints.push_back(kp);
          kp_ready = 0;
          kp_index++;
        }
      }

      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return keypoints;
}

std::vector<keypoints_t> load_keypoints(const std::string &data_path) {
  char keypoints_csv[1000] = {0};
  strcat(keypoints_csv, data_path.c_str());
  strcat(keypoints_csv, "/keypoints.csv");

  FILE *csv_file = fopen(keypoints_csv, "r");
  std::vector<keypoints_t> keypoints;

  char line[1024] = {0};
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }
    keypoints.push_back(parse_keypoints_line(line));
  }
  fclose(csv_file);

  return keypoints;
}

void keypoints_print(const keypoints_t &keypoints) {
  printf("nb_keypoints: %zu\n", keypoints.size());
  printf("keypoints:\n");
  for (size_t i = 0; i < keypoints.size(); i++) {
    printf("-- (%f, %f)\n", keypoints[i](0), keypoints[i](1));
  }
}

int check_jacobians(factor_t *factor,
                    const int param_idx,
                    const std::string &jac_name,
                    const real_t step_size,
                    const real_t threshold) {
  // Calculate baseline
  factor->eval();
  const vecx_t e = factor->residuals;
  const matx_t J = factor->jacobians[param_idx];

  // Numerical diff
  param_t *param = factor->params[param_idx];
  matx_t fdiff = zeros(e.rows(), param->local_size);
  for (size_t i = 0; i < param->local_size; i++) {
    // Perturb and evaluate
    param->perturb(i, step_size);
    factor->eval();
    auto e_prime = factor->residuals;
    param->perturb(i, -step_size);

    // Forward finite difference
    fdiff.block(0, i, e.rows(), 1) = (e_prime - e) / step_size;
  }

  return check_jacobian(jac_name, fdiff, J, threshold, true);
}

} // namespace proto

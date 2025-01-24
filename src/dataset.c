#include "dataset.h"

static int
parse_pose_data(const int i, const int j, const char *entry, pose_t *poses) {
  switch (j) {
    case 0:
      poses[i].ts = strtol(entry, NULL, 10);
      break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      poses[i].data[j - 1] = strtod(entry, NULL);
      break;
    default:
      return -1;
  }

  return 0;
}

/**
 * Load poses from file `fp`. The number of poses in file
 * will be outputted to `num_poses`.
 */
pose_t *load_poses(const char *fp, int *num_poses) {
  assert(fp != NULL);
  assert(num_poses != NULL);

  // Obtain number of rows and columns in dsv data
  int num_rows = dsv_rows(fp);
  int num_cols = dsv_cols(fp, ',');
  if (num_rows == -1 || num_cols == -1) {
    return NULL;
  }

  // Initialize memory for pose data
  *num_poses = num_rows;
  pose_t *poses = MALLOC(pose_t, num_rows);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    free(poses);
    return NULL;
  }

  // Loop through data
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  // Loop through data line by line
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    // Ignore if comment line
    if (line[0] == '#') {
      continue;
    }

    // Iterate through values in line separated by commas
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        if (parse_pose_data(row_idx, col_idx, entry, poses) != 0) {
          return NULL;
        }
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;

      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  // Clean up
  fclose(infile);

  return poses;
}

/**
 * Associate pose data
 */
int **assoc_pose_data(pose_t *gnd_poses,
                      size_t num_gnd_poses,
                      pose_t *est_poses,
                      size_t num_est_poses,
                      double threshold,
                      size_t *num_matches) {
  assert(gnd_poses != NULL);
  assert(est_poses != NULL);
  assert(num_gnd_poses != 0);
  assert(num_est_poses != 0);

  size_t gnd_idx = 0;
  size_t est_idx = 0;
  size_t k_end =
      (num_gnd_poses > num_est_poses) ? num_est_poses : num_gnd_poses;

  size_t match_idx = 0;
  int **matches = MALLOC(int *, k_end);

  while ((gnd_idx + 1) < num_gnd_poses && (est_idx + 1) < num_est_poses) {
    // Calculate time difference between ground truth and
    // estimate
    double gnd_k_time = ts2sec(gnd_poses[gnd_idx].ts);
    double est_k_time = ts2sec(est_poses[est_idx].ts);
    double t_k_diff = fabs(gnd_k_time - est_k_time);

    // Check to see if next ground truth timestamp forms
    // a smaller time diff
    double t_kp1_diff = threshold;
    if ((gnd_idx + 1) < num_gnd_poses) {
      double gnd_kp1_time = ts2sec(gnd_poses[gnd_idx + 1].ts);
      t_kp1_diff = fabs(gnd_kp1_time - est_k_time);
    }

    // Conditions to call this pair (ground truth and
    // estimate) a match
    int threshold_met = t_k_diff < threshold;
    int smallest_diff = t_k_diff < t_kp1_diff;

    // Mark pairs as a match or increment appropriate
    // indices
    if (threshold_met && smallest_diff) {
      matches[match_idx] = MALLOC(int, 2);
      matches[match_idx][0] = gnd_idx;
      matches[match_idx][1] = est_idx;
      match_idx++;

      gnd_idx++;
      est_idx++;

    } else if (gnd_k_time > est_k_time) {
      est_idx++;

    } else if (gnd_k_time < est_k_time) {
      gnd_idx++;
    }
  }

  // Clean up
  if (match_idx == 0) {
    free(matches);
    matches = NULL;
  }

  *num_matches = match_idx;
  return matches;
}

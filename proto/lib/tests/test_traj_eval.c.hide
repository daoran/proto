#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#define MAX_LINE_LENGTH 9046

char *malloc_string(const char *s) {
  char *retval = malloc(sizeof(char) * strlen(s) + 1);
  strcpy(retval, s);
  return retval;
}

int dsv_rows(const char *fp) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  /* Loop through lines */
  int nb_rows = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      nb_rows++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return nb_rows;
}

int dsv_cols(const char *fp, const char delim) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  /* Get line that isn't the header */
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  /* Parse line to obtain number of elements */
  int nb_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < MAX_LINE_LENGTH; i++) {
    if (line[i] == delim) {
      found_separator = 1;
      nb_elements++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return (found_separator) ? nb_elements : -1;
}

char **dsv_fields(const char *fp, const char delim, int *nb_fields) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Get last header line */
  char field_line[MAX_LINE_LENGTH] = {0};
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    } else {
      strcpy(field_line, line);
    }
  }

  /* Parse fields */
  *nb_fields = dsv_cols(fp, delim);
  char **fields = malloc(sizeof(char *) * *nb_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    /* Ignore # and ' ' */
    if (c == '#' || c == delim) {
      continue;
    }

    if (c == ',' || c == '\n') {
      /* Add field name to fields */
      fields[field_idx] = malloc_string(field_name);
      memset(field_name, '\0', sizeof(char) * 100);
      field_idx++;
    } else {
      /* Append field name */
      field_name[strlen(field_name)] = c;
    }
  }

  /* Cleanup */
  fclose(infile);

  return fields;
}

double **dsv_data(const char *fp, const char delim, int *nb_rows, int *nb_cols) {
  assert(fp != NULL);

  /* Obtain number of rows and columns in dsv data */
  *nb_rows = dsv_rows(fp);
  *nb_cols = dsv_cols(fp, delim);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for dsv data */
  double **data = malloc(sizeof(double *) * *nb_rows);
  for (int i = 0; i < *nb_rows; i++) {
    data[i] = malloc(sizeof(double) * *nb_cols);
  }

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  /* Loop through data line by line */
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[row_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  /* Clean up */
  fclose(infile);

  return data;
}

typedef struct pose_t {
  uint64_t ts;
  double r[3];
  double q[4];
} pose_t;

void pose_print(const pose_t *pose) {
  printf("ts: [%ld] ", pose->ts);
  printf("r: [%f, %f, %f] ", pose->r[0], pose->r[1], pose->r[2]);
  printf("q: [%f, %f, %f, %f]\n", pose->q[0], pose->q[1], pose->q[2], pose->q[3]);
}

typedef int (*load_cb_t)(const int i, const int j, const char *entry, void *data);

pose_t *load_poses(const char *fp, int *nb_poses, load_cb_t cb) {
  assert(fp != NULL);
  assert(nb_poses != NULL);

  /* Obtain number of rows and columns in dsv data */
  int nb_rows = dsv_rows(fp);
  int nb_cols = dsv_cols(fp, ',');
  if (nb_rows == -1 || nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for pose data */
  *nb_poses = nb_rows;
  pose_t *poses = malloc(sizeof(pose_t) * nb_rows);

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  /* Loop through data line by line */
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        if (cb(row_idx, col_idx, entry, poses) != 0) {
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

  /* Clean up */
  fclose(infile);

  return poses;
}

int pose_data_cb(const int i, const int j, const char *entry, void *data) {
  pose_t *poses = (pose_t *) data;

  switch (j) {
  /* Timestamp */
  case 0:
    poses[i].ts = strtol(entry, NULL, 0);
    break;
  /* Translation */
  case 1:
  case 2:
  case 3:
    poses[i].r[j-1] = strtod(entry, NULL);
    break;
  /* Orientation */
  case 4:
  case 5:
  case 6:
  case 7:
    poses[i].q[j-4] = strtod(entry, NULL);
    break;
  }

  return 0;
}

double ts2sec(const uint64_t ts) {
  return ts * 1e-9;
}

int **associate_data(pose_t *gnd_poses, size_t nb_gnd_poses,
                     pose_t *est_poses, size_t nb_est_poses,
                     double threshold, size_t *nb_matches) {
  assert(gnd_poses != NULL);
  assert(est_poses != NULL);
  assert(nb_gnd_poses != 0);
  assert(nb_est_poses != 0);

  size_t gnd_idx = 0;
  size_t est_idx = 0;
  size_t k_end = (nb_gnd_poses > nb_est_poses) ? nb_est_poses : nb_gnd_poses;

  size_t match_idx = 0;
  int **matches = malloc(sizeof(int *) * k_end);

  while((gnd_idx + 1) < nb_gnd_poses && (est_idx + 1) < nb_est_poses) {
    /* Calculate time difference between ground truth and estimate */
    double gnd_k_time = ts2sec(gnd_poses[gnd_idx].ts);
    double est_k_time = ts2sec(est_poses[est_idx].ts);
    double t_k_diff = fabs(gnd_k_time - est_k_time);

    /* Check to see if next ground truth timestamp forms a smaller time diff */
    double t_kp1_diff = threshold;
    if ((gnd_idx + 1) < nb_gnd_poses) {
      double gnd_kp1_time = ts2sec(gnd_poses[gnd_idx + 1].ts);
      t_kp1_diff = fabs(gnd_kp1_time - est_k_time);
    }

    /* Conditions to call this pair (ground truth and estimate) a match */
    int threshold_met = t_k_diff < threshold;
    int smallest_diff = t_k_diff < t_kp1_diff;

    /* Mark pairs as a match or increment appropriate indicies */
    if (threshold_met && smallest_diff) {
      matches[match_idx] = malloc(sizeof(int) * 2);
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

  /* Clean up */
  if (match_idx == 0) {
    free(matches);
    matches = NULL;
  }

  *nb_matches = match_idx;
  return matches;
}

int main() {
  const double threshold = 0.01;
  const char *matches_fpath = "./gnd_est_matches.csv";
  const char *gnd_data_path = "test_data/euroc/MH01_groundtruth.csv";
  const char *est_data_path = "test_data/euroc/MH01_estimate.csv";

  /* Load ground-truth poses */
  int nb_gnd_poses = 0;
  pose_t *gnd_poses = load_poses(gnd_data_path, &nb_gnd_poses, &pose_data_cb);

  /* Load estimate poses */
  int nb_est_poses = 0;
  pose_t *est_poses = load_poses(est_data_path, &nb_est_poses, &pose_data_cb);

  /* Associate data */
  size_t nb_matches = 0;
  int **matches = associate_data(gnd_poses, nb_gnd_poses,
                                 est_poses, nb_est_poses,
                                 threshold, &nb_matches);
  printf("Time Associated:\n");
  printf(" - [%s]\n", gnd_data_path);
  printf(" - [%s]\n", est_data_path);
  printf("threshold:  %.4f [s]\n", threshold);
  printf("nb_matches: %ld\n", nb_matches);

  /* Save matches to file */
  FILE *matches_csv = fopen(matches_fpath, "w");
  fprintf(matches_csv, "#gnd_idx,est_idx\n");
  for (size_t i = 0; i < nb_matches; i++) {
    uint64_t gnd_ts = gnd_poses[matches[i][0]].ts;
    uint64_t est_ts = est_poses[matches[i][1]].ts;
    double t_diff = fabs(ts2sec(gnd_ts - est_ts));
    if (t_diff > threshold) {
      printf("ERROR! Time difference > threshold!\n");
      printf("ground_truth_index: %d\n", matches[i][0]);
      printf("estimate_index: %d\n", matches[i][1]);
      break;
    }
    fprintf(matches_csv, "%d,%d\n", matches[i][0], matches[i][1]);
  }
  fclose(matches_csv);

  /* Clean up */
  for (size_t i = 0; i < nb_matches; i++) {
    free(matches[i]);
  }
  free(matches);
  free(gnd_poses);
  free(est_poses);

  return 0;
}

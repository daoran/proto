#include "munit.h"
#include "dataset.h"
#include "stb_ds.h"

int test_assoc_pose_data(void) {
  const double threshold = 0.01;
  const char *matches_fpath = "./gnd_est_matches.csv";
  const char *gnd_data_path = "./test_data/euroc/MH01_groundtruth.csv";
  const char *est_data_path = "./test_data/euroc/MH01_estimate.csv";

  // Load ground-truth poses
  int num_gnd_poses = 0;
  pose_t *gnd_poses = load_poses(gnd_data_path, &num_gnd_poses);
  printf("num_gnd_poses: %d\n", num_gnd_poses);

  // Load estimate poses
  int num_est_poses = 0;
  pose_t *est_poses = load_poses(est_data_path, &num_est_poses);
  printf("num_est_poses: %d\n", num_est_poses);

  // Associate data
  size_t num_matches = 0;
  int **matches = assoc_pose_data(gnd_poses,
                                  num_gnd_poses,
                                  est_poses,
                                  num_est_poses,
                                  threshold,
                                  &num_matches);
  printf("Time Associated:\n");
  printf(" - [%s]\n", gnd_data_path);
  printf(" - [%s]\n", est_data_path);
  printf("threshold:  %.4f [s]\n", threshold);
  printf("num_matches: %ld\n", num_matches);

  // Save matches to file
  FILE *matches_csv = fopen(matches_fpath, "w");
  fprintf(matches_csv, "#gnd_idx,est_idx\n");
  for (size_t i = 0; i < num_matches; i++) {
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

  // Clean up
  for (size_t i = 0; i < num_matches; i++) {
    free(matches[i]);
  }
  free(matches);
  free(gnd_poses);
  free(est_poses);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_assoc_pose_data);
}
MU_RUN_TESTS(test_suite)

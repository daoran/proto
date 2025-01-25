// #ifdef EUROC_UNITTEST
//
// #include <stdio.h>
// #include <math.h>
//
// // UNITESTS GLOBAL VARIABLES
// static int nb_tests = 0;
// static int nb_passed = 0;
// static int nb_failed = 0;
//
// #define ENABLE_TERM_COLORS 0
// #if ENABLE_TERM_COLORS == 1
// #define TERM_RED "\x1B[1;31m"
// #define TERM_GRN "\x1B[1;32m"
// #define TERM_WHT "\x1B[1;37m"
// #define TERM_NRM "\x1B[1;0m"
// #else
// #define TERM_RED
// #define TERM_GRN
// #define TERM_WHT
// #define TERM_NRM
// #endif
//
// /**
//  * Run unittests
//  * @param[in] test_name Test name
//  * @param[in] test_ptr Pointer to unittest
//  */
// void run_test(const char *test_name, int (*test_ptr)(void)) {
//   if ((*test_ptr)() == 0) {
//     printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
//     fflush(stdout);
//     nb_passed++;
//   } else {
//     printf(TERM_RED "FAILED!\n" TERM_NRM);
//     fflush(stdout);
//     nb_failed++;
//   }
//   nb_tests++;
// }
//
// /**
//  * Add unittest
//  * @param[in] TEST Test function
//  */
// #define TEST(TEST_FN) run_test(#TEST_FN, TEST_FN);
//
// /**
//  * Unit-test assert
//  * @param[in] TEST Test condition
//  */
// #define TEST_ASSERT(TEST)                                                      \
//   do {                                                                         \
//     if ((TEST) == 0) {                                                         \
//       printf(TERM_RED "ERROR!" TERM_NRM " [%s:%d] %s FAILED!\n",               \
//              __func__,                                                         \
//              __LINE__,                                                         \
//              #TEST);                                                           \
//       return -1;                                                               \
//     }                                                                          \
//   } while (0)
//
// /**
//  * Compare floats
//  */
// int fltcmp(const float x, const float y) {
//   if (fabs(x - y) < 1e-10) {
//     return 0;
//   } else if (x > y) {
//     return 1;
//   }
//
//   return -1;
// }
//
// int test_euroc_imu_load(void) {
//   const char *data_dir = "/data/euroc/imu_april/mav0/imu0";
//   euroc_imu_t *data = euroc_imu_load(data_dir);
//   // euroc_imu_print(data);
//   euroc_imu_free(data);
//   return 0;
// }
//
// int test_euroc_camera_load(void) {
//   const char *data_dir = "/data/euroc/imu_april/mav0/cam0";
//   euroc_camera_t *data = euroc_camera_load(data_dir, 1);
//   // euroc_camera_print(data);
//   euroc_camera_free(data);
//   return 0;
// }
//
// int test_euroc_ground_truth_load(void) {
//   const char *data_dir = "/data/euroc/V1_01/mav0/state_groundtruth_estimate0";
//   euroc_ground_truth_t *data = euroc_ground_truth_load(data_dir);
//   // euroc_ground_truth_print(data);
//   euroc_ground_truth_free(data);
//   return 0;
// }
//
// int test_euroc_data_load(void) {
//   const char *data_dir = "/data/euroc/V1_01";
//   euroc_data_t *data = euroc_data_load(data_dir);
//   euroc_data_free(data);
//   return 0;
// }
//
// int test_euroc_calib_target_load(void) {
//   const char *config_path = "/data/euroc/imu_april/april_6x6.yaml";
//   euroc_calib_target_t *data = euroc_calib_target_load(config_path);
//   // euroc_calib_target_print(data);
//   euroc_calib_target_free(data);
//   return 0;
// }
//
// int test_euroc_calib_load(void) {
//   const char *config_path = "/data/euroc/imu_april";
//   euroc_calib_t *data = euroc_calib_load(config_path);
//   euroc_calib_free(data);
//   return 0;
// }
//
// int main(int argc, char *argv[]) {
//   TEST(test_euroc_imu_load);
//   TEST(test_euroc_camera_load);
//   TEST(test_euroc_ground_truth_load);
//   TEST(test_euroc_data_load);
//   TEST(test_euroc_calib_target_load);
//   TEST(test_euroc_calib_load);
//   return (nb_failed) ? -1 : 0;
// }
//
// #endif // EUROC_UNITTEST

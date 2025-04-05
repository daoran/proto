#include "munit.h"
#include "xyz_gimbal.h"
#include "xyz_gnuplot.h"

int test_gimbal(void) {
  // Gimbal model
  gimbal_model_t model;
  gimbal_model_setup(&model);

  // Gimbal controller
  gimbal_ctrl_t ctrl;
  gimbal_ctrl_setup(&ctrl);

  // Simulate gimbal
  const real_t sp[3] = {0.1, 0.2, 0.3};
  const real_t dt = 0.001;
  const real_t t_end = 3.0;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  real_t *time_vals = calloc(N, sizeof(real_t));
  real_t *roll_vals = calloc(N, sizeof(real_t));
  real_t *pitch_vals = calloc(N, sizeof(real_t));
  real_t *yaw_vals = calloc(N, sizeof(real_t));

  while (idx < N) {
    const real_t pv[3] = {model.x[0], model.x[2], model.x[4]};

    real_t u[3] = {0};
    gimbal_ctrl_update(&ctrl, sp, pv, dt, u);
    gimbal_model_update(&model, u, dt);

    time_vals[idx] = t;
    roll_vals[idx] = model.x[0];
    pitch_vals[idx] = model.x[2];
    yaw_vals[idx] = model.x[4];

    t += dt;
    idx += 1;
  }

  // Plot
  int debug = 0;
  if (debug) {
    FILE *g = gnuplot_init();
    gnuplot_send(g, "set title 'Gimbal Attitude'");
    gnuplot_send_xy(g, "$r", time_vals, roll_vals, N);
    gnuplot_send_xy(g, "$p", time_vals, pitch_vals, N);
    gnuplot_send_xy(g, "$y", time_vals, yaw_vals, N);
    gnuplot_send(g, "plot $r with lines, $p with lines, $y with lines");
    gnuplot_close(g);
  }

  // Clean up
  free(time_vals);
  free(roll_vals);
  free(pitch_vals);
  free(yaw_vals);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_gimbal);
}
MU_RUN_TESTS(test_suite)

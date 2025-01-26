#include "munit.h"
#include "xyz_mav.h"

static void test_setup_mav(mav_model_t *mav) {
  // clang-format off
  const real_t x[12] = {
    // Attitude [rad]
    0.0, 0.0, 0.0,
    // Angular Velocity [rad / s]
    0.0, 0.0, 0.0,
    // Position [m]
    0.0, 0.0, 0.0,
    // Linear velocity [m / s]
    0.0, 0.0, 0.0
  };
  // clang-format on
  const real_t inertia[3] = {0.0963, 0.0963, 0.1927}; // Moment of inertia
  const real_t kr = 0.1;                              // Rotation drag constant
  const real_t kt = 0.2; // Translation drag constant
  const real_t l = 0.9;  // Arm Length
  const real_t d = 1.0;  // Drag constant
  const real_t m = 1.0;  // Mass
  const real_t g = 9.81; // Gravitational constant
  mav_model_setup(mav, x, inertia, kr, kt, l, d, m, g);
}

int test_mav_att_ctrl(void) {
  mav_model_t mav;
  test_setup_mav(&mav);

  mav_att_ctrl_t mav_att_ctrl;
  mav_pos_ctrl_t mav_pos_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_pos_ctrl_setup(&mav_pos_ctrl);

  const real_t att_sp[4] = {0.1, 0.2, -0.2, 0.0}; // roll, pitch, yaw, thrust
  const real_t dt = 0.001;
  const real_t t_end = 0.5;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  mav_model_telem_t *telem = mav_model_telem_malloc();

  while (idx < N) {
    const real_t att_pv[3] = {mav.x[0], mav.x[1], mav.x[2]};

    real_t u[4] = {0};
    mav_att_ctrl_update(&mav_att_ctrl, att_sp, att_pv, dt, u);
    mav_model_update(&mav, u, dt);
    mav_model_telem_update(telem, &mav, t);

    t += dt;
    idx += 1;
  }

  int debug = 0;
  if (debug) {
    mav_model_telem_plot(telem);
  }
  mav_model_telem_free(telem);

  return 0;
}

int test_mav_vel_ctrl(void) {
  mav_model_t mav;
  test_setup_mav(&mav);

  mav_att_ctrl_t mav_att_ctrl;
  mav_vel_ctrl_t mav_vel_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_vel_ctrl_setup(&mav_vel_ctrl);

  const real_t vel_sp[4] = {0.1, 0.2, 1.0, 0.0}; // vx, vy, vz, yaw
  const real_t dt = 0.001;
  const real_t t_end = 10.0;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  mav_model_telem_t *telem = mav_model_telem_malloc();

  while (idx < N) {
    const real_t vel_pv[4] = {mav.x[9], mav.x[10], mav.x[11], mav.x[2]};
    const real_t att_pv[3] = {mav.x[0], mav.x[1], mav.x[2]};

    real_t att_sp[4] = {0};
    real_t u[4] = {0};
    mav_vel_ctrl_update(&mav_vel_ctrl, vel_sp, vel_pv, dt, att_sp);
    mav_att_ctrl_update(&mav_att_ctrl, att_sp, att_pv, dt, u);
    mav_model_update(&mav, u, dt);
    mav_model_telem_update(telem, &mav, t);

    t += dt;
    idx += 1;
  }

  int debug = 0;
  if (debug) {
    mav_model_telem_plot(telem);
  }
  mav_model_telem_free(telem);

  return 0;
}

int test_mav_pos_ctrl(void) {
  mav_model_t mav;
  test_setup_mav(&mav);

  mav_att_ctrl_t mav_att_ctrl;
  mav_vel_ctrl_t mav_vel_ctrl;
  mav_pos_ctrl_t mav_pos_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_vel_ctrl_setup(&mav_vel_ctrl);
  mav_pos_ctrl_setup(&mav_pos_ctrl);

  const real_t pos_sp[4] = {10.0, 10.0, 5.0, 0.5}; // x, y, z, yaw
  const real_t dt = 0.001;
  const real_t t_end = 10.0;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  mav_model_telem_t *telem = mav_model_telem_malloc();

  while (idx < N) {
    const real_t pos_pv[4] = {mav.x[6], mav.x[7], mav.x[8], mav.x[2]};
    const real_t vel_pv[4] = {mav.x[9], mav.x[10], mav.x[11], mav.x[2]};
    const real_t att_pv[3] = {mav.x[0], mav.x[1], mav.x[2]};

    real_t vel_sp[4] = {0};
    real_t att_sp[4] = {0};
    real_t u[4] = {0};
    mav_pos_ctrl_update(&mav_pos_ctrl, pos_sp, pos_pv, dt, vel_sp);
    mav_vel_ctrl_update(&mav_vel_ctrl, vel_sp, vel_pv, dt, att_sp);
    mav_att_ctrl_update(&mav_att_ctrl, att_sp, att_pv, dt, u);
    mav_model_update(&mav, u, dt);
    mav_model_telem_update(telem, &mav, t);

    t += dt;
    idx += 1;
  }

  int debug = 0;
  if (debug) {
    mav_model_telem_plot(telem);
  }
  mav_model_telem_free(telem);

  return 0;
}

int test_mav_waypoints(void) {
  // Setup MAV model
  mav_model_t mav;
  test_setup_mav(&mav);

  // Setup MAV controllers
  mav_att_ctrl_t mav_att_ctrl;
  mav_vel_ctrl_t mav_vel_ctrl;
  mav_pos_ctrl_t mav_pos_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_vel_ctrl_setup(&mav_vel_ctrl);
  mav_pos_ctrl_setup(&mav_pos_ctrl);

  // Setup waypoints
  real_t waypoints[8][4] = {{0, 0, 1, 0},
                            {1, 1, 1, 0},
                            {1, -1, 1, 0},
                            {-1, -1, 1, 0},
                            {-1, 1, 1, 0},
                            {1, 1, 1, 0},
                            {0, 0, 1, 0},
                            {0, 0, 1, 1.0}};
  mav_waypoints_t *wps = mav_waypoints_malloc();
  for (int i = 0; i < 8; i++) {
    mav_waypoints_add(wps, waypoints[i]);
  }

  // Simulate
  const real_t dt = 0.001;
  const real_t t_end = 60.0;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  mav_model_telem_t *telem = mav_model_telem_malloc();

  while (idx < N) {
    const real_t pos_pv[4] = {mav.x[6], mav.x[7], mav.x[8], mav.x[2]};
    const real_t vel_pv[4] = {mav.x[9], mav.x[10], mav.x[11], mav.x[2]};
    const real_t att_pv[3] = {mav.x[0], mav.x[1], mav.x[2]};

    real_t pos_sp[4] = {0};
    mav_waypoints_update(wps, pos_pv, dt, pos_sp);

    real_t vel_sp[4] = {0};
    real_t att_sp[4] = {0};
    real_t u[4] = {0};
    mav_pos_ctrl_update(&mav_pos_ctrl, pos_sp, pos_pv, dt, vel_sp);
    mav_vel_ctrl_update(&mav_vel_ctrl, vel_sp, vel_pv, dt, att_sp);
    mav_att_ctrl_update(&mav_att_ctrl, att_sp, att_pv, dt, u);
    mav_model_update(&mav, u, dt);

    if (idx % 50 == 0) {
      mav_model_telem_update(telem, &mav, t);
    }

    t += dt;
    idx += 1;
  }

  // Plot and clean up
  int debug = 0;
  if (debug) {
    mav_model_telem_plot(telem);
    mav_model_telem_plot_xy(telem);
  }
  mav_model_telem_free(telem);
  mav_waypoints_free(wps);

  // gui_t gui;
  // gui.window_title = "Test";
  // gui.window_width = 640;
  // gui.window_height = 480;
  // gui_setup(&gui);
  // gui_loop(&gui);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_mav_att_ctrl);
  MU_ADD_TEST(test_mav_vel_ctrl);
  MU_ADD_TEST(test_mav_pos_ctrl);
  MU_ADD_TEST(test_mav_waypoints);
}
MU_RUN_TESTS(test_suite)

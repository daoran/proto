#include "xyz_mav.h"

void mav_model_setup(mav_model_t *mav,
                     const real_t x[12],
                     const real_t inertia[3],
                     const real_t kr,
                     const real_t kt,
                     const real_t l,
                     const real_t d,
                     const real_t m,
                     const real_t g) {
  vec_copy(x, 12, mav->x);            // State
  vec_copy(inertia, 3, mav->inertia); // Moment of inertia
  mav->kr = kr;                       // Rotation drag constant
  mav->kt = kt;                       // Translation drag constant
  mav->l = l;                         // Arm length
  mav->d = d;                         // Drag co-efficient
  mav->m = m;                         // Mass
  mav->g = g;                         // Gravitational constant
}

void mav_model_print_state(const mav_model_t *mav, const real_t time) {
  printf("time: %f, ", time);
  printf("pos: [%f, %f, %f], ", mav->x[6], mav->x[7], mav->x[8]);
  printf("att: [%f, %f, %f], ", mav->x[0], mav->x[1], mav->x[2]);
  printf("vel: [%f, %f, %f], ", mav->x[9], mav->x[10], mav->x[11]);
  printf("\n");
}

void mav_model_update(mav_model_t *mav, const real_t u[4], const real_t dt) {
  // Map out previous state
  // -- Attitude
  const real_t ph = mav->x[0];
  const real_t th = mav->x[1];
  const real_t ps = mav->x[2];
  // -- Angular velocity
  const real_t p = mav->x[3];
  const real_t q = mav->x[4];
  const real_t r = mav->x[5];
  // -- Velocity
  const real_t vx = mav->x[9];
  const real_t vy = mav->x[10];
  const real_t vz = mav->x[11];

  // Map out constants
  const real_t Ix = mav->inertia[0];
  const real_t Iy = mav->inertia[1];
  const real_t Iz = mav->inertia[2];
  const real_t kr = mav->kr;
  const real_t kt = mav->kt;
  const real_t m = mav->m;
  const real_t mr = 1.0 / m;
  const real_t g = mav->g;

  // Convert motor inputs to angular p, q, r and total thrust
  // clang-format off
  real_t A[4 * 4] = {
    1.0,          1.0,     1.0,   1.0,
    0.0,      -mav->l,     0.0,   mav->l,
    -mav->l,      0.0,  mav->l,   0.0,
    -mav->d,   mav->d,  -mav->d,  mav->d
  };
  // clang-format on

  // tau = A * u
  const real_t mt = 5.0; // Max-thrust
  const real_t s[4] = {mt * u[0], mt * u[1], mt * u[2], mt * u[3]};
  const real_t tauf = A[0] * s[0] + A[1] * s[1] + A[2] * s[2] + A[3] * s[3];
  const real_t taup = A[4] * s[0] + A[5] * s[1] + A[6] * s[2] + A[7] * s[3];
  const real_t tauq = A[8] * s[0] + A[9] * s[1] + A[10] * s[2] + A[11] * s[3];
  const real_t taur = A[12] * s[0] + A[13] * s[1] + A[14] * s[2] + A[15] * s[3];

  // Update state
  const real_t cph = cos(ph);
  const real_t sph = sin(ph);
  const real_t cth = cos(th);
  const real_t sth = sin(th);
  const real_t tth = tan(th);
  const real_t cps = cos(ps);
  const real_t sps = sin(ps);

  real_t *x = mav->x;
  // -- Attitude
  x[0] += (p + q * sph * tth + r * cos(ph) * tth) * dt;
  x[1] += (q * cph - r * sph) * dt;
  x[2] += ((1 / cth) * (q * sph + r * cph)) * dt;
  // s[2] = wrapToPi(s[2]);
  // -- Angular velocity
  x[3] += (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
  x[4] += (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
  x[5] += (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;
  // -- Position
  x[6] += vx * dt;
  x[7] += vy * dt;
  x[8] += vz * dt;
  // -- Linear velocity
  x[9] += ((-kt * vx / m) + mr * (cph * sth * cps + sph * sps) * tauf) * dt;
  x[10] += ((-kt * vy / m) + mr * (cph * sth * sps - sph * cps) * tauf) * dt;
  x[11] += (-(kt * vz / m) + mr * (cph * cth) * tauf - g) * dt;
}

void mav_model_attitude(const mav_model_t *mav, real_t rpy[3]) {
  rpy[0] = mav->x[0];
  rpy[1] = mav->x[1];
  rpy[2] = mav->x[2];
}

void mav_model_angular_velocity(const mav_model_t *mav, real_t pqr[3]) {
  pqr[0] = mav->x[3];
  pqr[1] = mav->x[4];
  pqr[2] = mav->x[5];
}

void mav_model_position(const mav_model_t *mav, real_t pos[3]) {
  pos[0] = mav->x[6];
  pos[1] = mav->x[7];
  pos[2] = mav->x[8];
}

void mav_model_velocity(const mav_model_t *mav, real_t vel[3]) {
  vel[0] = mav->x[9];
  vel[1] = mav->x[10];
  vel[2] = mav->x[11];
}

mav_model_telem_t *mav_model_telem_malloc() {
  mav_model_telem_t *telem = MALLOC(mav_model_telem_t, 1);

  telem->num_events = 0;
  telem->time = NULL;
  telem->roll = NULL;
  telem->pitch = NULL;
  telem->yaw = NULL;
  telem->wx = NULL;
  telem->wy = NULL;
  telem->wz = NULL;
  telem->x = NULL;
  telem->y = NULL;
  telem->z = NULL;
  telem->vx = NULL;
  telem->vy = NULL;
  telem->vz = NULL;

  return telem;
}

void mav_model_telem_free(mav_model_telem_t *telem) {
  free(telem->time);
  free(telem->roll);
  free(telem->pitch);
  free(telem->yaw);
  free(telem->wx);
  free(telem->wy);
  free(telem->wz);
  free(telem->x);
  free(telem->y);
  free(telem->z);
  free(telem->vx);
  free(telem->vy);
  free(telem->vz);
  free(telem);
}

void mav_model_telem_update(mav_model_telem_t *telem,
                            const mav_model_t *mav,
                            const real_t time) {
  const int idx = telem->num_events;
  const int ns = idx + 1;

  telem->time = REALLOC(telem->time, real_t, ns);
  telem->roll = REALLOC(telem->roll, real_t, ns);
  telem->pitch = REALLOC(telem->pitch, real_t, ns);
  telem->yaw = REALLOC(telem->yaw, real_t, ns);
  telem->wx = REALLOC(telem->wx, real_t, ns);
  telem->wy = REALLOC(telem->wy, real_t, ns);
  telem->wz = REALLOC(telem->wz, real_t, ns);
  telem->x = REALLOC(telem->x, real_t, ns);
  telem->y = REALLOC(telem->y, real_t, ns);
  telem->z = REALLOC(telem->z, real_t, ns);
  telem->vx = REALLOC(telem->vx, real_t, ns);
  telem->vy = REALLOC(telem->vy, real_t, ns);
  telem->vz = REALLOC(telem->vz, real_t, ns);

  telem->num_events = ns;
  telem->time[idx] = time;
  telem->roll[idx] = rad2deg(mav->x[0]);
  telem->pitch[idx] = rad2deg(mav->x[1]);
  telem->yaw[idx] = rad2deg(mav->x[2]);
  telem->wx[idx] = mav->x[3];
  telem->wy[idx] = mav->x[4];
  telem->wz[idx] = mav->x[5];
  telem->x[idx] = mav->x[6];
  telem->y[idx] = mav->x[7];
  telem->z[idx] = mav->x[8];
  telem->vx[idx] = mav->x[9];
  telem->vy[idx] = mav->x[10];
  telem->vz[idx] = mav->x[11];
}

void mav_model_telem_plot(const mav_model_telem_t *telem) {
  // Plot
  FILE *g = gnuplot_init();

  // -- Plot settings
  gnuplot_send(g, "set multiplot layout 3,1");
  gnuplot_send(g, "set colorsequence classic");
  gnuplot_send(g, "set style line 1 lt 1 pt -1 lw 1");
  gnuplot_send(g, "set style line 2 lt 2 pt -1 lw 1");
  gnuplot_send(g, "set style line 3 lt 3 pt -1 lw 1");

  // -- Attitude
  gnuplot_send(g, "set title 'Attitude'");
  gnuplot_send(g, "set xlabel 'Time [s]'");
  gnuplot_send(g, "set ylabel 'Attitude [deg]'");
  gnuplot_send_xy(g, "$roll", telem->time, telem->roll, telem->num_events);
  gnuplot_send_xy(g, "$pitch", telem->time, telem->pitch, telem->num_events);
  gnuplot_send_xy(g, "$yaw", telem->time, telem->yaw, telem->num_events);
  gnuplot_send(g, "plot $roll with lines, $pitch with lines, $yaw with lines");

  // -- Displacement
  gnuplot_send(g, "set title 'Displacement'");
  gnuplot_send(g, "set xlabel 'Time [s]'");
  gnuplot_send(g, "set ylabel 'Displacement [m]'");
  gnuplot_send_xy(g, "$x", telem->time, telem->x, telem->num_events);
  gnuplot_send_xy(g, "$y", telem->time, telem->y, telem->num_events);
  gnuplot_send_xy(g, "$z", telem->time, telem->z, telem->num_events);
  gnuplot_send(g, "plot $x with lines, $y with lines, $z with lines");

  // -- Velocity
  gnuplot_send(g, "set title 'Velocity'");
  gnuplot_send(g, "set xlabel 'Time [s]'");
  gnuplot_send(g, "set ylabel 'Velocity [m/s]'");
  gnuplot_send_xy(g, "$vx", telem->time, telem->vx, telem->num_events);
  gnuplot_send_xy(g, "$vy", telem->time, telem->vy, telem->num_events);
  gnuplot_send_xy(g, "$vz", telem->time, telem->vz, telem->num_events);
  gnuplot_send(g, "plot $vx with lines, $vy with lines, $vz with lines");

  // Clean up
  gnuplot_close(g);
}

void mav_model_telem_plot_xy(const mav_model_telem_t *telem) {
  FILE *g = gnuplot_init();

  real_t x_min = vec_min(telem->x, telem->num_events);
  real_t x_max = vec_max(telem->x, telem->num_events);
  real_t y_min = vec_min(telem->y, telem->num_events);
  real_t y_max = vec_max(telem->y, telem->num_events);
  real_t x_pad = (x_max - x_min) * 0.1;
  real_t y_pad = (x_max - x_min) * 0.1;

  gnuplot_send(g, "set colorsequence classic");
  gnuplot_send_xy(g, "$DATA", telem->x, telem->y, telem->num_events);
  gnuplot_xrange(g, x_min - x_pad, x_max + x_pad);
  gnuplot_yrange(g, y_min - y_pad, y_max + y_pad);
  gnuplot_send(g, "set xlabel 'X [m]'");
  gnuplot_send(g, "set ylabel 'Y [m]'");
  gnuplot_send(g, "plot $DATA with lines lt 1 lw 2");

  gnuplot_close(g);
}

void mav_att_ctrl_setup(mav_att_ctrl_t *ctrl) {
  ctrl->dt = 0;
  pid_ctrl_setup(&ctrl->roll, 100.0, 0.0, 5.0);
  pid_ctrl_setup(&ctrl->pitch, 100.0, 0.0, 5.0);
  pid_ctrl_setup(&ctrl->yaw, 10.0, 0.0, 1.0);
  zeros(ctrl->u, 4, 1);
}

void mav_att_ctrl_update(mav_att_ctrl_t *ctrl,
                         const real_t sp[4],
                         const real_t pv[3],
                         const real_t dt,
                         real_t u[4]) {
  // Check rate
  ctrl->dt += dt;
  if (ctrl->dt < 0.001) {
    // Return previous command
    u[0] = ctrl->u[0];
    u[1] = ctrl->u[1];
    u[2] = ctrl->u[2];
    u[3] = ctrl->u[3];
    return;
  }

  // Roll, pitch, yaw and thrust
  const real_t error_yaw = wrap_pi(sp[2] - pv[2]);
  const real_t r = pid_ctrl_update(&ctrl->roll, sp[0], pv[0], ctrl->dt);
  const real_t p = pid_ctrl_update(&ctrl->pitch, sp[1], pv[1], ctrl->dt);
  const real_t y = pid_ctrl_update(&ctrl->yaw, error_yaw, 0.0, ctrl->dt);
  const real_t t = clip_value(sp[3], 0.0, 1.0);

  // Map roll, pitch, yaw and thrust to motor outputs
  u[0] = clip_value(-p - y + t, 0.0, 1.0);
  u[1] = clip_value(-r + y + t, 0.0, 1.0);
  u[2] = clip_value(p - y + t, 0.0, 1.0);
  u[3] = clip_value(r + y + t, 0.0, 1.0);

  // Keep track of control action
  ctrl->u[0] = u[0];
  ctrl->u[1] = u[1];
  ctrl->u[2] = u[2];
  ctrl->u[3] = u[3];
  ctrl->dt = 0.0; // Reset dt
}

void mav_vel_ctrl_setup(mav_vel_ctrl_t *ctrl) {
  ctrl->dt = 0;
  pid_ctrl_setup(&ctrl->vx, 1.0, 0.0, 0.05);
  pid_ctrl_setup(&ctrl->vy, 1.0, 0.0, 0.05);
  pid_ctrl_setup(&ctrl->vz, 10.0, 0.0, 0.0);
  zeros(ctrl->u, 4, 1);
}

void mav_vel_ctrl_update(mav_vel_ctrl_t *ctrl,
                         const real_t sp[4],
                         const real_t pv[4],
                         const real_t dt,
                         real_t u[4]) {
  // Check rate
  ctrl->dt += dt;
  if (ctrl->dt < 0.001) {
    // Return previous command
    u[0] = ctrl->u[0];
    u[1] = ctrl->u[1];
    u[2] = ctrl->u[2];
    u[3] = ctrl->u[3];
    return;
  }

  // Calculate RPY errors relative to quadrotor by incorporating yaw
  const real_t errors_W[3] = {sp[0] - pv[0], sp[1] - pv[1], sp[2] - pv[2]};
  const real_t ypr[3] = {pv[3], 0.0, 0.0};
  real_t C_WS[3 * 3] = {0};
  real_t C_SW[3 * 3] = {0};
  real_t errors[3] = {0};
  euler321(ypr, C_WS);
  mat_transpose(C_WS, 3, 3, C_SW);
  dot(C_SW, 3, 3, errors_W, 3, 1, errors);

  // Roll, pitch, yaw and thrust
  const real_t r = -pid_ctrl_update(&ctrl->vy, errors[1], 0.0, dt);
  const real_t p = pid_ctrl_update(&ctrl->vx, errors[0], 0.0, dt);
  const real_t y = sp[3];
  const real_t t = 0.5 + pid_ctrl_update(&ctrl->vz, errors[2], 0.0, dt);

  u[0] = clip_value(r, deg2rad(-20.0), deg2rad(20.0));
  u[1] = clip_value(p, deg2rad(-20.0), deg2rad(20.0));
  u[2] = y;
  u[3] = clip_value(t, 0.0, 1.0);

  // // Yaw first if threshold reached
  // if (fabs(sp[3] - pv[3]) > deg2rad(2)) {
  //   outputs[0] = 0.0;
  //   outputs[1] = 0.0;
  // }

  // Keep track of control action
  ctrl->u[0] = u[0];
  ctrl->u[1] = u[1];
  ctrl->u[2] = u[2];
  ctrl->u[3] = u[3];
  ctrl->dt = 0.0; // Reset dt
}

void mav_pos_ctrl_setup(mav_pos_ctrl_t *ctrl) {
  ctrl->dt = 0;
  pid_ctrl_setup(&ctrl->x, 0.5, 0.0, 0.05);
  pid_ctrl_setup(&ctrl->y, 0.5, 0.0, 0.05);
  pid_ctrl_setup(&ctrl->z, 1.0, 0.0, 0.1);
  zeros(ctrl->u, 4, 1);
}

void mav_pos_ctrl_update(mav_pos_ctrl_t *ctrl,
                         const real_t sp[4],
                         const real_t pv[4],
                         const real_t dt,
                         real_t u[4]) {
  // Check rate
  ctrl->dt += dt;
  if (ctrl->dt < 0.01) {
    // Return previous command
    u[0] = ctrl->u[0];
    u[1] = ctrl->u[1];
    u[2] = ctrl->u[2];
    u[3] = ctrl->u[3];
    return;
  }

  // Calculate RPY errors relative to quadrotor by incorporating yaw
  const real_t errors_W[3] = {sp[0] - pv[0], sp[1] - pv[1], sp[2] - pv[2]};
  const real_t ypr[3] = {pv[3], 0.0, 0.0};
  real_t C_WS[3 * 3] = {0};
  real_t C_SW[3 * 3] = {0};
  real_t errors[3] = {0};
  euler321(ypr, C_WS);
  mat_transpose(C_WS, 3, 3, C_SW);
  dot(C_SW, 3, 3, errors_W, 3, 1, errors);

  // Velocity commands
  const real_t vx = pid_ctrl_update(&ctrl->x, errors[0], 0.0, ctrl->dt);
  const real_t vy = pid_ctrl_update(&ctrl->y, errors[1], 0.0, ctrl->dt);
  const real_t vz = pid_ctrl_update(&ctrl->z, errors[2], 0.0, ctrl->dt);
  const real_t yaw = sp[3];

  u[0] = clip_value(vx, -2.5, 2.5);
  u[1] = clip_value(vy, -2.5, 2.5);
  u[2] = clip_value(vz, -5.0, 5.0);
  u[3] = yaw;

  // Keep track of control action
  ctrl->u[0] = u[0];
  ctrl->u[1] = u[1];
  ctrl->u[2] = u[2];
  ctrl->u[3] = u[3];
  ctrl->dt = 0.0;
}

mav_waypoints_t *mav_waypoints_malloc(void) {
  mav_waypoints_t *wps = MALLOC(mav_waypoints_t, 1);

  wps->num_waypoints = 0;
  wps->waypoints = NULL;
  wps->index = 0;

  wps->wait_mode = 0;
  wps->wait_time = 0.0;

  wps->threshold_dist = 0.1;
  wps->threshold_yaw = 0.1;
  wps->threshold_wait = 2.0;

  return wps;
}

void mav_waypoints_free(mav_waypoints_t *wps) {
  free(wps->waypoints);
  free(wps);
}

void mav_waypoints_print(const mav_waypoints_t *wps) {
  printf("num_waypoints: %d\n", wps->num_waypoints);
  for (int k = 0; k < wps->num_waypoints; k++) {
    const real_t x = wps->waypoints[k * 4 + 0];
    const real_t y = wps->waypoints[k * 4 + 1];
    const real_t z = wps->waypoints[k * 4 + 2];
    const real_t yaw = wps->waypoints[k * 4 + 3];
    printf("[%d]: (%.2f, %.2f, %.2f, %.2f)\n", k, x, y, z, yaw);
  }
}

int mav_waypoints_done(const mav_waypoints_t *wps) {
  return wps->index == wps->num_waypoints;
}

void mav_waypoints_add(mav_waypoints_t *wps, real_t wp[4]) {
  const int n = wps->num_waypoints;
  wps->waypoints = REALLOC(wps->waypoints, real_t, (n + 1) * 4);
  wps->waypoints[n * 4 + 0] = wp[0];
  wps->waypoints[n * 4 + 1] = wp[1];
  wps->waypoints[n * 4 + 2] = wp[2];
  wps->waypoints[n * 4 + 3] = wp[3];
  wps->num_waypoints++;
}

void mav_waypoints_target(const mav_waypoints_t *wps, real_t wp[4]) {
  if (mav_waypoints_done(wps) == 1) {
    wp[0] = wps->waypoints[(wps->index - 1) * 4 + 0];
    wp[1] = wps->waypoints[(wps->index - 1) * 4 + 1];
    wp[2] = wps->waypoints[(wps->index - 1) * 4 + 2];
    wp[3] = wps->waypoints[(wps->index - 1) * 4 + 3];
    return;
  }

  wp[0] = wps->waypoints[wps->index * 4 + 0];
  wp[1] = wps->waypoints[wps->index * 4 + 1];
  wp[2] = wps->waypoints[wps->index * 4 + 2];
  wp[3] = wps->waypoints[wps->index * 4 + 3];
}

int mav_waypoints_update(mav_waypoints_t *wps,
                         const real_t state[4],
                         const real_t dt,
                         real_t wp[4]) {
  assert(wps->index >= 0 && wps->index <= wps->num_waypoints);

  // Check if waypoints completed - return last waypoint
  if (mav_waypoints_done(wps) == 1) {
    mav_waypoints_target(wps, wp);
    return -1;
  }

  // Check if in wait mode - gap between waypoints
  if (wps->wait_mode == 1 && (wps->wait_time >= wps->threshold_wait)) {
    // Go to next waypoint
    wps->index++;
    mav_waypoints_target(wps, wp);

    // Reset wait mode
    wps->wait_mode = 0;
    wps->wait_time = 0.0;
    return 0;
  }

  // Check if we're close to current waypoint
  mav_waypoints_target(wps, wp);
  const real_t dx = state[0] - wp[0];
  const real_t dy = state[1] - wp[1];
  const real_t dz = state[2] - wp[2];
  const real_t diff_dist = sqrt(dx * dx + dy * dy + dz * dz);
  const real_t diff_yaw = fabs(state[3] - wp[3]);
  if (diff_dist < wps->threshold_dist && diff_yaw < wps->threshold_yaw) {
    // Transition to wait mode
    wps->wait_mode = 1;
    wps->wait_time += dt;
  } else {
    // Reset wait mode
    wps->wait_mode = 0;
    wps->wait_time = 0.0;
  }

  return 0;
}

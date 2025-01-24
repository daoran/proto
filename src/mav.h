#pragma once

#include "macros.h"
#include "math.h"
#include "control.h"
#include "gnuplot.h"

/** MAV Model **/
typedef struct mav_model_t {
  real_t x[12];      // State
  real_t inertia[3]; // Moment of inertia
  real_t kr;         // Rotation drag constant
  real_t kt;         // Translation drag constant
  real_t l;          // Arm length
  real_t d;          // Drag
  real_t m;          // Mass
  real_t g;          // Gravitational constant
} mav_model_t;

void mav_model_setup(mav_model_t *mav,
                     const real_t x[12],
                     const real_t inertia[3],
                     const real_t kr,
                     const real_t kt,
                     const real_t l,
                     const real_t d,
                     const real_t m,
                     const real_t g);
void mav_model_attitude(const mav_model_t *mav, real_t rpy[3]);
void mav_model_angular_velocity(const mav_model_t *mav, real_t pqr[3]);
void mav_model_position(const mav_model_t *mav, real_t pos[3]);
void mav_model_velocity(const mav_model_t *mav, real_t vel[3]);
void mav_model_print_state(const mav_model_t *mav, const real_t time);
void mav_model_update(mav_model_t *mav, const real_t u[4], const real_t dt);

/** MAV Model Telemetry **/
typedef struct mav_model_telem_t {
  int num_events;
  real_t *time;

  real_t *roll;
  real_t *pitch;
  real_t *yaw;

  real_t *wx;
  real_t *wy;
  real_t *wz;

  real_t *x;
  real_t *y;
  real_t *z;

  real_t *vx;
  real_t *vy;
  real_t *vz;

} mav_model_telem_t;

mav_model_telem_t *mav_model_telem_malloc(void);
void mav_model_telem_free(mav_model_telem_t *telem);
void mav_model_telem_update(mav_model_telem_t *telem,
                            const mav_model_t *mav,
                            const real_t time);
void mav_model_telem_plot(const mav_model_telem_t *telem);
void mav_model_telem_plot_xy(const mav_model_telem_t *telem);

/** MAV Attitude Controller **/
typedef struct mav_att_ctrl_t {
  real_t dt;
  pid_ctrl_t roll;
  pid_ctrl_t pitch;
  pid_ctrl_t yaw;
  real_t u[4];
} mav_att_ctrl_t;

void mav_att_ctrl_setup(mav_att_ctrl_t *ctrl);
void mav_att_ctrl_update(mav_att_ctrl_t *ctrl,
                         const real_t sp[4],
                         const real_t pv[3],
                         const real_t dt,
                         real_t u[4]);

/** MAV Velocity Controller **/
typedef struct mav_vel_ctrl_t {
  real_t dt;
  pid_ctrl_t vx;
  pid_ctrl_t vy;
  pid_ctrl_t vz;
  real_t u[4];
} mav_vel_ctrl_t;

void mav_vel_ctrl_setup(mav_vel_ctrl_t *ctrl);
void mav_vel_ctrl_update(mav_vel_ctrl_t *ctrl,
                         const real_t sp[4],
                         const real_t pv[4],
                         const real_t dt,
                         real_t u[4]);

/** MAV Position Controller **/
typedef struct mav_pos_ctrl_t {
  real_t dt;
  pid_ctrl_t x;
  pid_ctrl_t y;
  pid_ctrl_t z;
  real_t u[4];
} mav_pos_ctrl_t;

void mav_pos_ctrl_setup(mav_pos_ctrl_t *ctrl);
void mav_pos_ctrl_update(mav_pos_ctrl_t *ctrl,
                         const real_t sp[4],
                         const real_t pv[4],
                         const real_t dt,
                         real_t u[4]);

/** MAV Waypoints **/
typedef struct mav_waypoints_t {
  int num_waypoints;
  real_t *waypoints;
  int index;

  int wait_mode;
  real_t wait_time;

  real_t threshold_dist;
  real_t threshold_yaw;
  real_t threshold_wait;
} mav_waypoints_t;

mav_waypoints_t *mav_waypoints_malloc(void);
void mav_waypoints_free(mav_waypoints_t *ctrl);
void mav_waypoints_print(const mav_waypoints_t *wps);
int mav_waypoints_done(const mav_waypoints_t *wps);
void mav_waypoints_add(mav_waypoints_t *wps, real_t wp[4]);
void mav_waypoints_target(const mav_waypoints_t *wps, real_t wp[4]);
int mav_waypoints_update(mav_waypoints_t *wps,
                         const real_t state[4],
                         const real_t dt,
                         real_t wp[4]);

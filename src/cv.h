#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include "macros.h"
#include "logging.h"
#include "math.h"

/*******************************************************************************
 * CV
 ******************************************************************************/

///////////
// IMAGE //
///////////

typedef struct image_t {
  int width;
  int height;
  int channels;
  uint8_t *data;
} image_t;

void image_setup(image_t *img,
                 const int width,
                 const int height,
                 uint8_t *data);
image_t *image_load(const char *file_path);
void image_print_properties(const image_t *img);
void image_free(image_t *img);

////////////
// RADTAN //
////////////

void radtan4_distort(const real_t params[4],
                     const real_t p_in[2],
                     real_t p_out[2]);
void radtan4_undistort(const real_t params[4],
                       const real_t p_in[2],
                       real_t p_out[2]);
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]);
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]);

//////////
// EQUI //
//////////

void equi4_distort(const real_t params[4],
                   const real_t p_in[2],
                   real_t p_out[2]);
void equi4_undistort(const real_t params[4],
                     const real_t p_in[2],
                     real_t p_out[2]);
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]);
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]);

/////////////
// PINHOLE //
/////////////

typedef void (*project_func_t)(const real_t *params,
                               const real_t p_C[3],
                               real_t z_out[2]);

typedef void (*back_project_func_t)(const real_t *params,
                                    const real_t z[2],
                                    real_t bearing[3]);

typedef void (*undistort_func_t)(const real_t *params,
                                 const real_t z_in[2],
                                 real_t z_out[2]);

real_t pinhole_focal(const int image_width, const real_t fov);
void pinhole_K(const real_t params[4], real_t K[3 * 3]);
void pinhole_projection_matrix(const real_t params[4],
                               const real_t T[4 * 4],
                               real_t P[3 * 4]);
void pinhole_project(const real_t params[4], const real_t p_C[3], real_t z[2]);
void pinhole_point_jacobian(const real_t params[4], real_t J_point[2 * 2]);
void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]);

/////////////////////
// PINHOLE-RADTAN4 //
/////////////////////

void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t z[2]);
void pinhole_radtan4_undistort(const real_t params[8],
                               const real_t z_in[2],
                               real_t z_out[2]);
void pinhole_radtan4_back_project(const real_t params[8],
                                  const real_t z[2],
                                  real_t ray[3]);
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]);
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]);

///////////////////
// PINHOLE-EQUI4 //
///////////////////

void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t z[2]);
void pinhole_equi4_undistort(const real_t params[8],
                             const real_t z_in[2],
                             real_t z_out[2]);
void pinhole_equi4_back_project(const real_t params[8],
                                const real_t z[2],
                                real_t ray[3]);
void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]);
void pinhole_equi4_params_jacobian(const real_t params[8],
                                   const real_t p_C[3],
                                   real_t J[2 * 8]);

//////////////
// GEOMETRY //
//////////////

void linear_triangulation(const real_t P_i[3 * 4],
                          const real_t P_j[3 * 4],
                          const real_t z_i[2],
                          const real_t z_j[2],
                          real_t p[3]);

int homography_find(const real_t *pts_i,
                    const real_t *pts_j,
                    const int num_points,
                    real_t H[3 * 3]);

int homography_pose(const real_t *proj_params,
                    const real_t *img_pts,
                    const real_t *obj_pts,
                    const int N,
                    real_t T_CF[4 * 4]);

int p3p_kneip(const real_t features[3][3],
              const real_t points[3][3],
              real_t solutions[4][4 * 4]);

int solvepnp(const real_t proj_params[4],
             const real_t *img_pts,
             const real_t *obj_pts,
             const int N,
             real_t T_CO[4 * 4]);

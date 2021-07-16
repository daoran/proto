#ifndef _GUI_H_
#define _GUI_H_

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
// #include <GLFW/glfw3.h>

#define SDL_DISABLE_IMMINTRIN_H 1
#include <SDL2/SDL.h>

#include "proto.h"

/******************************************************************************
 * UTILS
 ******************************************************************************/

GLfloat gl_deg2rad(const GLfloat d);
GLfloat gl_rad2deg(const GLfloat r);
void gl_print_vector(const char *prefix, const GLfloat *x, const int length);
void gl_print_matrix(const char *prefix,
                     const GLfloat *A,
                     const int nb_rows,
                     const int nb_cols);
int gl_equals(const GLfloat *A,
              const GLfloat *B,
              const int nb_rows,
              const int nb_cols,
              const GLfloat tol);
void gl_matf_set(GLfloat *A,
                 const int m,
                 const int n,
                 const int i,
                 const int j,
                 const GLfloat val);
GLfloat gl_matf_val(
    const GLfloat *A, const int m, const int n, const int i, const int j);
void gl_copy(const GLfloat *src, const int m, const int n, GLfloat *dest);
void gl_transpose(const real_t *A, size_t m, size_t n, real_t *A_t);
void gl_zeros(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_ones(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_eye(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_vec2f(GLfloat *v, const GLfloat x, const GLfloat y);
void gl_vec3f(GLfloat *v, const GLfloat x, const GLfloat y, const GLfloat z);
void gl_vec4f(GLfloat *v,
              const GLfloat x,
              const GLfloat y,
              const GLfloat z,
              const GLfloat w);
void gl_vec3f_cross(const GLfloat u[3], const GLfloat v[3], GLfloat n[3]);

void gl_add(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C);
void gl_sub(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C);
void gl_dot(const GLfloat *A,
            const int A_m,
            const int A_n,
            const GLfloat *B,
            const int B_m,
            const int B_n,
            GLfloat *C);
void gl_scale(GLfloat factor, GLfloat *A, const int nb_rows, const int nb_cols);
GLfloat gl_norm(const GLfloat *x, const int size);
void gl_normalize(GLfloat *x, const int size);

void gl_perspective(const GLfloat fov,
                    const GLfloat aspect,
                    const GLfloat near,
                    const GLfloat far,
                    GLfloat P[4 * 4]);
void gl_lookat(const GLfloat eye[3],
               const GLfloat at[3],
               const GLfloat up[3],
               GLfloat V[4 * 4]);

/******************************************************************************
 * SHADER
 ******************************************************************************/

GLuint shader_compile(const char *shader_src, const int type);
GLuint shaders_link(const GLuint vertex_shader,
                    const GLuint fragment_shader,
                    const GLuint geometry_shader);

/******************************************************************************
 * GL PROGRAM
 ******************************************************************************/

typedef struct gl_entity_t {
  GLfloat T[4 * 4];

  GLint program_id;
  GLuint vao;
  GLuint vbo;
  GLuint ebo;
} gl_entity_t;

GLuint gl_prog_setup(const char *vs_src,
                     const char *fs_src,
                     const char *gs_src);

int gl_prog_set_int(const GLint id, const char *k, const GLint v);
int gl_prog_set_vec2i(const GLint id, const char *k, const GLint v[2]);
int gl_prog_set_vec3i(const GLint id, const char *k, const GLint v[3]);
int gl_prog_set_vec4i(const GLint id, const char *k, const GLint v[4]);

int gl_prog_set_float(const GLint id, const char *k, const GLfloat v);
int gl_prog_set_vec2f(const GLint id, const char *k, const GLfloat v[2]);
int gl_prog_set_vec3f(const GLint id, const char *k, const GLfloat v[3]);
int gl_prog_set_vec4f(const GLint id, const char *k, const GLfloat v[4]);
int gl_prog_set_mat2f(const GLint id, const char *k, const GLfloat v[2 * 2]);
int gl_prog_set_mat3f(const GLint id, const char *k, const GLfloat v[3 * 3]);
int gl_prog_set_mat4f(const GLint id, const char *k, const GLfloat v[4 * 4]);

/******************************************************************************
 * GL-CAMERA
 ******************************************************************************/

typedef struct gl_camera_t {
  int *window_width;
  int *window_height;

  GLfloat focal[3];
  GLfloat world_up[3];
  GLfloat position[3];
  GLfloat right[3];
  GLfloat up[3];
  GLfloat front[3];
  GLfloat yaw;
  GLfloat pitch;
  GLfloat radius;

  GLfloat fov;
  GLfloat near;
  GLfloat far;

  GLfloat P[4 * 4]; // Projection matrix
  GLfloat V[4 * 4]; // View matrix
} gl_camera_t;

void gl_camera_setup(gl_camera_t *camera,
                     int *screen_width,
                     int *screen_height);
void gl_camera_update(gl_camera_t *camera);
void gl_camera_rotate(gl_camera_t *camera,
                      const float factor,
                      const float dx,
                      const float dy);
void gl_camera_pan(gl_camera_t *camera,
                   const float factor,
                   const float dx,
                   const float dy);
void gl_camera_zoom(gl_camera_t *camera,
                    const float factor,
                    const float dx,
                    const float dy);

/******************************************************************************
 * GUI
 ******************************************************************************/

typedef struct gui_t {
  int screen_width;
  int screen_height;

  SDL_Window *window;
  char *window_title;
  int window_width;
  int window_height;
  int loop;

  gl_camera_t camera;
  GLfloat movement_speed;
  GLfloat mouse_sensitivity;

  int left_click;
  int right_click;
  int last_cursor_set;
  float last_cursor_x;
  float last_cursor_y;
} gui_t;

void gui_window_callback(gui_t *gui, const SDL_Event event);
void gui_keyboard_callback(gui_t *gui, const SDL_Event event);
void gui_mouse_callback(gui_t *gui, const SDL_Event event);
void gui_event_handler(gui_t *gui);

void gui_setup(gui_t *gui);
void gui_reset(gui_t *gui);
void gui_loop(gui_t *gui);

void gl_cube_setup(gl_entity_t *entity, GLfloat pos[3]);
void gl_cube_cleanup(const gl_entity_t *entity);
void gl_cube_draw(const gl_entity_t *entity, const gl_camera_t *camera);

void gl_camera_frame_setup(gl_entity_t *entity);
void gl_camera_frame_cleanup(const gl_entity_t *entity);
void gl_camera_frame_draw(const gl_entity_t *entity, const gl_camera_t *camera);

void gl_axis_frame_setup(gl_entity_t *entity);
void gl_axis_frame_cleanup(const gl_entity_t *entity);
void gl_axis_frame_draw(const gl_entity_t *entity, const gl_camera_t *camera);

void gl_grid_setup(gl_entity_t *entity);
void gl_grid_cleanup(const gl_entity_t *entity);
void gl_grid_draw(const gl_entity_t *entity, const gl_camera_t *camera);

#endif /* _GUI_H_ */

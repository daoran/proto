#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>
#include <libgen.h>

#include "stb_image.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>

#include <ft2build.h>
#include FT_FREETYPE_H

/******************************************************************************
 * MACROS
 *****************************************************************************/

/**
 * Mark variable unused.
 * @param[in] expr Variable to mark as unused
 */
#ifndef UNUSED
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)
#endif

/**
 * Return max between a and b
 */
#ifndef MAX
#define MAX(a, b) a > b ? a : b
#endif

/**
 * Return min between a and b
 */
#ifndef MIN
#define MIN(a, b) a < b ? a : b;
#endif

/**
 * Fatal
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef FATAL
#define FATAL(...)                                                             \
  do {                                                                         \
    fprintf(stderr, "[FATAL] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);                                                                 \
  exit(-1)
#endif

/**
 * Log error
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef LOG_ERROR
#define LOG_ERROR(...)                                                         \
  do {                                                                         \
    fprintf(stderr, "[ERROR] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0)
#endif

/** Macro that adds the ability to switch between C / C++ style mallocs */
#ifdef __cplusplus

#ifndef MALLOC
#define MALLOC(TYPE, N) (TYPE *) malloc(sizeof(TYPE) * (N))
#endif

#ifndef REALLOC
#define REALLOC(PTR, TYPE, N) (TYPE *) realloc(PTR, sizeof(TYPE) * (N));
#endif

#ifndef CALLOC
#define CALLOC(TYPE, N) (TYPE *) calloc((N), sizeof(TYPE))
#endif

#else

#ifndef MALLOC
#define MALLOC(TYPE, N) malloc(sizeof(TYPE) * (N))
#endif

#ifndef REALLOC
#define REALLOC(PTR, TYPE, N) realloc(PTR, sizeof(TYPE) * (N))
#endif

#ifndef CALLOC
#define CALLOC(TYPE, N) calloc((N), sizeof(TYPE))
#endif

#endif

/******************************************************************************
 * OPENGL UTILS
 *****************************************************************************/

typedef GLuint gl_uint_t;
typedef GLint gl_int_t;
typedef GLfloat gl_float_t;
typedef GLenum gl_enum_t;

typedef struct {
  gl_float_t x;
  gl_float_t y;
  gl_float_t z;
} gl_vec3_t;

typedef struct {
  gl_float_t w;
  gl_float_t x;
  gl_float_t y;
  gl_float_t z;
} gl_quat_t;

typedef struct {
  gl_float_t r;
  gl_float_t g;
  gl_float_t b;
} gl_color_t;

typedef struct {
  gl_int_t x;
  gl_int_t y;
  gl_int_t w;
  gl_int_t h;
} gl_bounds_t;

typedef struct {
  gl_vec3_t pos;
  gl_quat_t quat;
} gl_pose_t;

char *load_file(const char *fp);

gl_enum_t gl_check_error(const char *file, const int line);
#define GL_CHECK_ERROR() gl_check_error(__FILE__, __LINE__)

gl_float_t gl_randf(const gl_float_t a, const gl_float_t b);
gl_float_t gl_deg2rad(const gl_float_t d);
gl_float_t gl_rad2deg(const gl_float_t r);
gl_float_t gl_rad2deg(const gl_float_t r);
void gl_print_vector(const char *prefix, const gl_float_t *x, const int length);
void gl_print_matrix(const char *prefix,
                     const gl_float_t *A,
                     const int num_rows,
                     const int num_cols);
int gl_equals(const gl_float_t *A,
              const gl_float_t *B,
              const int num_rows,
              const int num_cols,
              const gl_float_t tol);
void gl_mat_set(gl_float_t *A,
                const int m,
                const int n,
                const int i,
                const int j,
                const gl_float_t val);
gl_float_t gl_mat_val(const gl_float_t *A,
                      const int m,
                      const int n,
                      const int i,
                      const int j);
void gl_copy(const gl_float_t *src, const int m, const int n, gl_float_t *dest);
void gl_transpose(const gl_float_t *A, size_t m, size_t n, gl_float_t *A_t);
void gl_zeros(gl_float_t *A, const int num_rows, const int num_cols);
void gl_ones(gl_float_t *A, const int num_rows, const int num_cols);
void gl_eye(gl_float_t *A, const int num_rows, const int num_cols);
void gl_vec2(gl_float_t *v, const gl_float_t x, const gl_float_t y);
void gl_vec3(gl_float_t *v,
             const gl_float_t x,
             const gl_float_t y,
             const gl_float_t z);
void gl_vec4(gl_float_t *v,
             const gl_float_t x,
             const gl_float_t y,
             const gl_float_t z,
             const gl_float_t w);
void gl_vec3_cross(const gl_float_t u[3],
                   const gl_float_t v[3],
                   gl_float_t n[3]);

void gl_add(const gl_float_t *A,
            const gl_float_t *B,
            const int num_rows,
            const int num_cols,
            gl_float_t *C);
void gl_sub(const gl_float_t *A,
            const gl_float_t *B,
            const int num_rows,
            const int num_cols,
            gl_float_t *C);
void gl_dot(const gl_float_t *A,
            const int A_m,
            const int A_n,
            const gl_float_t *B,
            const int B_m,
            const int B_n,
            gl_float_t *C);
void gl_scale(gl_float_t factor,
              gl_float_t *A,
              const int num_rows,
              const int num_cols);
gl_float_t gl_norm(const gl_float_t *x, const int size);
void gl_normalize(gl_float_t *x, const int size);

void gl_perspective(const gl_float_t fov,
                    const gl_float_t aspect,
                    const gl_float_t near,
                    const gl_float_t far,
                    gl_float_t P[4 * 4]);
void gl_ortho(const gl_float_t w, const gl_float_t h, gl_float_t P[4 * 4]);
void gl_lookat(const gl_float_t eye[3],
               const gl_float_t at[3],
               const gl_float_t up[3],
               gl_float_t V[4 * 4]);
void gl_quat2rot(const gl_quat_t *q, gl_float_t C[3 * 3]);
void gl_rot2quat(const gl_float_t C[3 * 3], gl_quat_t *q);
void gl_tf2pose(const gl_float_t T[4 * 4], gl_pose_t *pose);
int gl_save_frame_buffer(const int width, const int height, const char *fp);

/******************************************************************************
 * GL-SHADER
 *****************************************************************************/

typedef struct gl_shader_t {
  gl_uint_t program_id;
  gl_uint_t texture_id;
  gl_uint_t VAO;
  gl_uint_t VBO;
  gl_uint_t EBO;
} gl_shader_t;

void gl_shader_setup(gl_shader_t *shader);
void gl_shader_cleanup(gl_shader_t *shader);
gl_uint_t gl_compile(const char *src, const int type);
gl_uint_t gl_link(const gl_uint_t vs, const gl_uint_t fs, const gl_uint_t gs);
gl_uint_t gl_shader(const char *vs_src, const char *fs_src, const char *gs_src);

int gl_set_color(const gl_int_t id, const char *k, const gl_color_t v);
int gl_set_int(const gl_int_t id, const char *k, const gl_int_t v);
int gl_set_float(const gl_int_t id, const char *k, const gl_float_t v);
int gl_set_vec2i(const gl_int_t id, const char *k, const gl_int_t v[2]);
int gl_set_vec3i(const gl_int_t id, const char *k, const gl_int_t v[3]);
int gl_set_vec4i(const gl_int_t id, const char *k, const gl_int_t v[4]);
int gl_set_vec2(const gl_int_t id, const char *k, const gl_float_t v[2]);
int gl_set_vec3(const gl_int_t id, const char *k, const gl_float_t v[3]);
int gl_set_vec4(const gl_int_t id, const char *k, const gl_float_t v[4]);
int gl_set_mat2(const gl_int_t id, const char *k, const gl_float_t v[2 * 2]);
int gl_set_mat3(const gl_int_t id, const char *k, const gl_float_t v[3 * 3]);
int gl_set_mat4(const gl_int_t id, const char *k, const gl_float_t v[4 * 4]);

/******************************************************************************
 * GL-CAMERA
 *****************************************************************************/

typedef enum { ORBIT, FPS } gl_view_mode_t;

typedef struct gl_camera_t {
  gl_view_mode_t view_mode;

  gl_float_t focal[3];
  gl_float_t world_up[3];
  gl_float_t position[3];
  gl_float_t right[3];
  gl_float_t up[3];
  gl_float_t front[3];
  gl_float_t yaw;
  gl_float_t pitch;
  gl_float_t radius;

  gl_float_t fov;
  gl_float_t fov_min;
  gl_float_t fov_max;
  gl_float_t near;
  gl_float_t far;

  gl_float_t P[4 * 4]; // Projection matrix
  gl_float_t V[4 * 4]; // View matrix
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
 *****************************************************************************/

void gui_setup(const char *window_title,
               const int window_width,
               const int window_height);
void gui_loop(void);

// RECT //////////////////////////////////////////////////////////////////////

void setup_rect_shader(gl_shader_t *rect);
void draw_rect(const gl_bounds_t *bounds, const gl_color_t *color);

// CUBE //////////////////////////////////////////////////////////////////////

void setup_cube_shader(gl_shader_t *cube);
void draw_cube(const gl_float_t T[4 * 4],
               const gl_float_t size,
               const gl_color_t color);

// FRUSTUM ///////////////////////////////////////////////////////////////////

void setup_frustum_shader(gl_shader_t *frustum);
void draw_frustum(const gl_float_t T[4 * 4],
                  const gl_float_t size,
                  const gl_color_t color,
                  const gl_float_t lw);

// AXES 3D ///////////////////////////////////////////////////////////////////

void setup_axes3d_shader(gl_shader_t *axes);
void draw_axes3d(const gl_float_t T[4 * 4],
                 const gl_float_t size,
                 const gl_float_t lw);

// GRID 3D ///////////////////////////////////////////////////////////////////

void setup_grid3d_shader(gl_shader_t *grid);
void draw_grid3d(const gl_float_t size,
                 const gl_float_t lw,
                 const gl_color_t color);

// POINTS 3D /////////////////////////////////////////////////////////////////

typedef struct gl_points3d {
  gl_uint_t VAO;
  gl_uint_t VBO;

  gl_float_t *points;
  size_t num_points;
  gl_float_t point_size;
} gl_points3d;

void setup_points3d_shader(gl_shader_t *points);
void draw_points3d(const gl_float_t *points_data,
                   const size_t num_points,
                   const gl_float_t size);

// LINE 3D ///////////////////////////////////////////////////////////////////

void setup_line3d_shader(gl_shader_t *line);
void draw_line3d(const gl_float_t *data,
                 const size_t num_points,
                 const gl_color_t color,
                 const gl_float_t lw);

// IMAGE /////////////////////////////////////////////////////////////////////

void setup_image_shader(gl_shader_t *shader);
void draw_image(const int x,
                const int y,
                const uint8_t *data,
                const int w,
                const int h,
                const int c);

// TEXT //////////////////////////////////////////////////////////////////////

typedef struct {
  gl_uint_t texture_id;
  gl_int_t size[2];    // Size of glyph
  gl_int_t bearing[2]; // Offset from baseline to left/top of glyph
  gl_uint_t offset;    // Offset to advance to next glyph
} gl_char_t;

void setup_text_shader(gl_shader_t *text);
void text_width_height(const char *s, gl_float_t *w, gl_float_t *h);
void draw_text(const char *s, const float x, const float y, const gl_color_t c);

// MESH //////////////////////////////////////////////////////////////////////

typedef struct gl_vertex_t {
  float position[3];
  float normal[3];
  float tex_coords[2];
  float tangent[3];
  float bitangent[3];
} gl_vertex_t;

typedef struct gl_texture_t {
  unsigned int id;
  char type[100];
  char path[100];
} gl_texture_t;

typedef struct gl_mesh_t {
  gl_vertex_t *vertices;
  unsigned int *indices;
  gl_texture_t *textures;

  int num_vertices;
  int num_indices;
  int num_textures;

  gl_uint_t VAO;
  gl_uint_t VBO;
  gl_uint_t EBO;
} gl_mesh_t;

void gl_mesh_setup(gl_mesh_t *mesh,
                   gl_vertex_t *vertices,
                   const int num_vertices,
                   unsigned int *indices,
                   const int num_indices,
                   gl_texture_t *textures,
                   const int num_textures);
void gl_mesh_draw(const gl_mesh_t *mesh, const gl_uint_t shader);

// MODEL /////////////////////////////////////////////////////////////////////

typedef struct gl_model_t {
  char model_dir[100];

  gl_float_t T[4 * 4];
  gl_uint_t program_id;

  gl_mesh_t *meshes;
  int num_meshes;

  int enable_gamma_correction;
} gl_model_t;

gl_model_t *gl_model_load(const char *model_path);
void gl_model_free(gl_model_t *model);
void gl_model_draw(const gl_model_t *model, const gl_camera_t *camera);

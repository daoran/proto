#ifndef GUI_H
#define GUI_H

#ifdef __cplusplus
#error "This module only supports C builds"
#endif

#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <libgen.h>

#include "stb_image.h"

// #include <GL/glew.h>
// #include <GL/gl.h>
// #include <GL/glu.h>
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
#define MALLOC(TYPE, N) (TYPE *) malloc(sizeof(TYPE) * (N));
#endif

#ifndef REALLOC
#define REALLOC(PTR, TYPE, N) (TYPE *) realloc(PTR, sizeof(TYPE) * (N));
#endif

#ifndef CALLOC
#define CALLOC(TYPE, N) (TYPE *) calloc((N), sizeof(TYPE));
#endif

#else

#ifndef MALLOC
#define MALLOC(TYPE, N) malloc(sizeof(TYPE) * (N));
#endif

#ifndef REALLOC
#define REALLOC(PTR, TYPE, N) realloc(PTR, sizeof(TYPE) * (N));
#endif

#ifndef CALLOC
#define CALLOC(TYPE, N) calloc((N), sizeof(TYPE));
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

int file_exists(const char *fp);
char *load_file(const char *fp);

gl_enum_t gl_check_error(const char *file, const int line);
#define GL_CHECK_ERROR() gl_check_error(__FILE__, __LINE__)

gl_float_t gl_randf(const gl_float_t a, const gl_float_t b);
gl_float_t gl_deg2rad(const gl_float_t d);
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
void gl_ortho(const gl_float_t left,
              const gl_float_t right,
              const gl_float_t bottom,
              const gl_float_t top,
              const gl_float_t znear,
              const gl_float_t zfar,
              gl_float_t P[4 * 4]);
void gl_lookat(const gl_float_t eye[3],
               const gl_float_t at[3],
               const gl_float_t up[3],
               gl_float_t V[4 * 4]);
int gl_save_frame_buffer(const int width, const int height, const char *fp);

/******************************************************************************
 * GL-ENTITY
 *****************************************************************************/

typedef struct gl_entity_t {
  gl_float_t T[4 * 4];
  gl_float_t P[4 * 4];

  gl_uint_t program_id;
  gl_uint_t VAO;
  gl_uint_t VBO;
  gl_uint_t EBO;
} gl_entity_t;

void gl_quat2rot(const gl_float_t q[4], gl_float_t C[3 * 3]);
void gl_rot2quat(const gl_float_t C[3 * 3], gl_float_t q[4]);

void gl_entity_setup(gl_entity_t *entity);
void gl_entity_cleanup(gl_entity_t *entity);
void gl_entity_set_position(gl_entity_t *entity, const gl_float_t pos[3]);
void gl_entity_get_position(gl_entity_t *entity, gl_float_t *pos);
void gl_entity_set_rotation(gl_entity_t *entity, const gl_float_t rot[3 * 3]);
void gl_entity_get_rotation(gl_entity_t *entity, gl_float_t *rot);
void gl_entity_set_quaternion(gl_entity_t *entity, const gl_float_t quat[4]);
void gl_entity_get_quaternion(gl_entity_t *entity, gl_float_t *quat);

/******************************************************************************
 * GL-SHADER
 *****************************************************************************/

gl_uint_t gl_shader_compile(const char *shader_src, const int type);
gl_uint_t gl_shaders_link(const gl_uint_t vertex_shader,
                          const gl_uint_t fragment_shader,
                          const gl_uint_t geometry_shader);

/******************************************************************************
 * GL-PROGRAM
 *****************************************************************************/

gl_uint_t gl_prog_setup(const char *vs_src,
                        const char *fs_src,
                        const char *gs_src);

int gl_prog_set_int(const gl_int_t id, const char *k, const gl_int_t v);
int gl_prog_set_vec2i(const gl_int_t id, const char *k, const gl_int_t v[2]);
int gl_prog_set_vec3i(const gl_int_t id, const char *k, const gl_int_t v[3]);
int gl_prog_set_vec4i(const gl_int_t id, const char *k, const gl_int_t v[4]);

int gl_prog_set_color(const gl_int_t id, const char *k, const gl_color_t v);
int gl_prog_set_float(const gl_int_t id, const char *k, const gl_float_t v);
int gl_prog_set_vec2(const gl_int_t id, const char *k, const gl_float_t v[2]);
int gl_prog_set_vec3(const gl_int_t id, const char *k, const gl_float_t v[3]);
int gl_prog_set_vec4(const gl_int_t id, const char *k, const gl_float_t v[4]);
int gl_prog_set_mat2(const gl_int_t id,
                     const char *k,
                     const gl_float_t v[2 * 2]);
int gl_prog_set_mat3(const gl_int_t id,
                     const char *k,
                     const gl_float_t v[3 * 3]);
int gl_prog_set_mat4(const gl_int_t id,
                     const char *k,
                     const gl_float_t v[4 * 4]);

/******************************************************************************
 * GL-CAMERA
 *****************************************************************************/

typedef enum { ORBIT, FPS } gl_view_mode_t;

typedef struct gl_camera_t {
  gl_view_mode_t view_mode;
  int *window_width;
  int *window_height;

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
 * GL-PRIMITIVES
 *****************************************************************************/

// GL RECT ///////////////////////////////////////////////////////////////////

typedef struct {
  gl_entity_t entity;
  gl_bounds_t bounds;
} gl_rect_t;
gl_rect_t *gl_rect_malloc(const gl_bounds_t bounds);
void gl_rect_free(gl_rect_t *rect);
void gl_rect_draw(const gl_rect_t *rect, const gl_camera_t *camera);

// GL CUBE ///////////////////////////////////////////////////////////////////

typedef struct {
  gl_entity_t entity;
  gl_float_t size;
  gl_float_t color[3];
  gl_float_t outline_color[3];
} gl_cube_t;
gl_cube_t *gl_cube_malloc(const gl_float_t size,
                          const gl_float_t pos[3],
                          const gl_float_t color[3],
                          const gl_float_t outline_color[3]);
void gl_cube_free(gl_cube_t *cube);
void gl_cube_draw(const gl_cube_t *cube, const gl_camera_t *camera);

// GL CAMERA FRAME ///////////////////////////////////////////////////////////

typedef struct {
  gl_entity_t entity;
  gl_float_t size;
  gl_float_t color[3];
  gl_float_t line_width;
} gl_cframe_t;
gl_cframe_t *gl_cframe_malloc(const gl_float_t pos[3],
                              const gl_float_t size,
                              const gl_float_t color[3]);
void gl_cframe_free(gl_cframe_t *cf);
void gl_cframe_draw(const gl_cframe_t *cf, const gl_camera_t *camera);

// GL AXES 3D ////////////////////////////////////////////////////////////////

typedef struct {
  gl_entity_t entity;
  gl_float_t size;
  gl_float_t line_width;
} gl_axes3d_t;
gl_axes3d_t *gl_axes3d_malloc(void);
void gl_axes3d_free(gl_axes3d_t *axes);
void gl_axes3d_draw(const gl_axes3d_t *axes, const gl_camera_t *camera);

// GL GRID ///////////////////////////////////////////////////////////////////

typedef struct {
  gl_entity_t entity;
  gl_float_t size;
  gl_float_t line_width;
} gl_grid3d_t;
gl_grid3d_t *gl_grid3d_malloc(void);
void gl_grid3d_free(gl_grid3d_t *grid);
void gl_grid3d_draw(const gl_grid3d_t *grid, const gl_camera_t *camera);

// GL POINTS /////////////////////////////////////////////////////////////////

typedef struct {
  gl_entity_t entity;
  gl_float_t *data;
  gl_uint_t num_points;
  gl_float_t size;
  gl_float_t color[3];
} gl_points3d_t;
gl_points3d_t *gl_points3d_malloc(const float *data,
                                  const size_t num_points,
                                  const gl_float_t size,
                                  const gl_float_t color[3]);
void gl_points3d_free(gl_points3d_t *points);
void gl_points3d_draw(const gl_points3d_t *points,
                      const gl_camera_t *camera,
                      const size_t num_points);

// GL LINE ///////////////////////////////////////////////////////////////////

typedef struct {
  gl_entity_t entity;
  gl_float_t *data;
  gl_int_t num_points;
  gl_float_t line_width;
  gl_float_t color[3];
} gl_line3d_t;
gl_line3d_t *gl_line3d_malloc(const gl_float_t *data,
                              const size_t num_points,
                              const gl_float_t line_width,
                              const gl_float_t color[3]);
void gl_line3d_free(gl_line3d_t *line);
void gl_line3d_draw(const gl_line3d_t *line, const gl_camera_t *camera);

// GL TEXT ///////////////////////////////////////////////////////////////////

typedef struct {
  unsigned int texture_id; // ID handle of the glyph texture
  unsigned int size[2];    // Size of glyph
  unsigned int bearing[2]; // Offset from baseline to left/top of glyph
  unsigned int offset;     // Offset to advance to next glyph
} gl_char_t;

typedef struct {
  gl_entity_t entity;
  gl_char_t data[128];
  gl_color_t color;
  gl_int_t size;
} gl_text_t;

void gl_char_print(const gl_char_t *ch);
void gl_text_setup(gl_text_t *text);
void gl_text_cleanup(gl_text_t *text);
gl_text_t *gl_text_malloc(void);
void gl_text_free(gl_text_t *text);
void gl_text_draw(gl_text_t *text,
                  gl_camera_t *camera,
                  const char *s,
                  float x,
                  const float y,
                  const float scale);

// GL IMAGE //////////////////////////////////////////////////////////////////

typedef struct {
  gl_uint_t program_id;
  gl_uint_t texture_id;
  gl_uint_t vao;
  gl_uint_t vbo;
  gl_uint_t ebo;
} gl_image_t;
gl_image_t *gl_image_malloc(void);
void gl_image_free(gl_image_t *image);
void gl_image_draw(gl_image_t *image);

/******************************************************************************
 * GL-MESH
 *****************************************************************************/

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

/******************************************************************************
 * GL-MODEL
 *****************************************************************************/

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

/******************************************************************************
 * GUI
 *****************************************************************************/

typedef struct gui_t {
  int screen_width;
  int screen_height;

  GLFWwindow *window;
  char *window_title;
  int window_width;
  int window_height;
  int loop;

  gl_camera_t camera;
  gl_float_t movement_speed;
  gl_float_t mouse_sensitivity;

  double cursor_x;
  double cursor_y;
  int left_click;
  int right_click;

  int last_cursor_set;
  float last_cursor_x;
  float last_cursor_y;
} gui_t;

void gui_setup(gui_t *gui);
void gui_reset(gui_t *gui);
void gui_loop(gui_t *gui);

typedef struct {
  gl_entity_t entity;
  gl_color_t color;
  gl_color_t color_hover;
  gl_color_t color_press;
  gl_bounds_t bounds;
  gl_text_t text;

  gl_uint_t program_id;
  gl_uint_t vao;
  gl_uint_t vbo;
  gl_uint_t ebo;
} ui_button_t;
void ui_button_setup(ui_button_t *button);
void ui_button_cleanup(const ui_button_t *button);
int ui_button_draw(const gui_t *gui, const ui_button_t *button);

#endif // GUI_H

#ifdef GUI_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#include <time.h>

#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif

#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#endif

/******************************************************************************
 * OPENGL UTILS
 *****************************************************************************/

/**
 * Check if file exists.
 * @returns 0 for success, -1 for failure
 */
int file_exists(const char *fp) { return access(fp, F_OK); }

/**
 * Read file contents in file path `fp`.
 * @returns
 * - Success: File contents
 * - Failure: NULL
 */
char *load_file(const char *fp) {
  assert(fp != NULL);
  FILE *f = fopen(fp, "rb");
  if (f == NULL) {
    return NULL;
  }

  fseek(f, 0, SEEK_END);
  long int len = ftell(f);
  fseek(f, 0, SEEK_SET);

  char *buf = MALLOC(char, len + 1);
  if (buf == NULL) {
    return NULL;
  }
  const ssize_t read = fread(buf, 1, len, f);
  if (read != len) {
    FATAL("Failed to read file [%s]\n", fp);
  }
  buf[len] = '\0';
  fclose(f);

  return buf;
}

gl_enum_t gl_check_error(const char *file, const int line) {
  gl_enum_t error_code;

  while ((error_code = glGetError()) != GL_NO_ERROR) {
    char error[1000] = {0};
    switch (error_code) {
      case GL_INVALID_ENUM:
        strcpy(error, "INVALID_ENUM");
        break;
      case GL_INVALID_VALUE:
        strcpy(error, "INVALID_VALUE");
        break;
      case GL_INVALID_OPERATION:
        strcpy(error, "INVALID_OPERATION");
        break;
      case GL_STACK_OVERFLOW:
        strcpy(error, "STACK_OVERFLOW");
        break;
      case GL_STACK_UNDERFLOW:
        strcpy(error, "STACK_UNDERFLOW");
        break;
      case GL_OUT_OF_MEMORY:
        strcpy(error, "OUT_OF_MEMORY");
        break;
      case GL_INVALID_FRAMEBUFFER_OPERATION:
        strcpy(error, "INVALID_FRAMEBUFFER_OPERATION");
        break;
    }
    printf("%s | %s:%d\n", error, file, line);
  }

  return error_code;
}

/**
 * Generate random number between a and b from a uniform distribution.
 * @returns Random number
 */
gl_float_t gl_randf(const gl_float_t a, const gl_float_t b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

gl_float_t gl_deg2rad(const gl_float_t d) { return d * M_PI / 180.0f; }

gl_float_t gl_rad2deg(const gl_float_t r) { return r * 180.0f / M_PI; }

void gl_print_vector(const char *prefix,
                     const gl_float_t *x,
                     const int length) {
  printf("%s: [", prefix);
  for (int i = 0; i < length; i++) {
    printf("%f", x[i]);
    if ((i + 1) != length) {
      printf(", ");
    }
  }
  printf("]\n");
}

void gl_print_matrix(const char *prefix,
                     const gl_float_t *A,
                     const int num_rows,
                     const int num_cols) {
  printf("%s:\n", prefix);
  for (int i = 0; i < num_rows; i++) {
    for (int j = 0; j < num_cols; j++) {
      printf("%f", A[i + (j * num_rows)]);
      if ((j + 1) != num_cols) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void gl_zeros(gl_float_t *A, const int num_rows, const int num_cols) {
  for (int i = 0; i < (num_rows * num_cols); i++) {
    A[i] = 0.0f;
  }
}

void gl_ones(gl_float_t *A, const int num_rows, const int num_cols) {
  for (int i = 0; i < (num_rows * num_cols); i++) {
    A[i] = 1.0f;
  }
}

void gl_eye(gl_float_t *A, const int num_rows, const int num_cols) {
  int idx = 0;
  for (int j = 0; j < num_cols; j++) {
    for (int i = 0; i < num_rows; i++) {
      A[idx++] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

void gl_vec2(gl_float_t *v, const gl_float_t x, const gl_float_t y) {
  v[0] = x;
  v[1] = y;
}

void gl_vec3(gl_float_t *v,
             const gl_float_t x,
             const gl_float_t y,
             const gl_float_t z) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
}

void gl_vec4(gl_float_t *v,
             const gl_float_t x,
             const gl_float_t y,
             const gl_float_t z,
             const gl_float_t w) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
  v[3] = w;
}

int gl_equals(const gl_float_t *A,
              const gl_float_t *B,
              const int num_rows,
              const int num_cols,
              const gl_float_t tol) {
  for (int i = 0; i < (num_rows * num_cols); i++) {
    if (fabs(A[i] - B[i]) > tol) {
      return 0;
    }
  }

  return 1;
}

void gl_mat_set(gl_float_t *A,
                const int m,
                const int n,
                const int i,
                const int j,
                const gl_float_t val) {
  UNUSED(n);
  A[i + (j * m)] = val;
}

gl_float_t gl_mat_val(const gl_float_t *A,
                      const int m,
                      const int n,
                      const int i,
                      const int j) {
  UNUSED(n);
  return A[i + (j * m)];
}

void gl_copy(const gl_float_t *src,
             const int m,
             const int n,
             gl_float_t *dest) {
  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

void gl_transpose(const gl_float_t *A, size_t m, size_t n, gl_float_t *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  int idx = 0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A_t[idx++] = gl_mat_val(A, m, n, i, j);
    }
  }
}

void gl_vec3_cross(const gl_float_t u[3],
                   const gl_float_t v[3],
                   gl_float_t n[3]) {
  assert(u);
  assert(v);
  assert(n);

  n[0] = u[1] * v[2] - u[2] * v[1];
  n[1] = u[2] * v[0] - u[0] * v[2];
  n[2] = u[0] * v[1] - u[1] * v[0];
}

void gl_add(const gl_float_t *A,
            const gl_float_t *B,
            const int num_rows,
            const int num_cols,
            gl_float_t *C) {
  for (int i = 0; i < (num_rows * num_cols); i++) {
    C[i] = A[i] + B[i];
  }
}

void gl_sub(const gl_float_t *A,
            const gl_float_t *B,
            const int num_rows,
            const int num_cols,
            gl_float_t *C) {
  for (int i = 0; i < (num_rows * num_cols); i++) {
    C[i] = A[i] - B[i];
  }
}

void gl_dot(const gl_float_t *A,
            const int A_m,
            const int A_n,
            const gl_float_t *B,
            const int B_m,
            const int B_n,
            gl_float_t *C) {
  assert(A != C && B != C);
  assert(A_n == B_m);

  int m = A_m;
  int n = B_n;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      for (int k = 0; k < A_n; k++) {
        C[i + (j * n)] += A[i + (k * A_n)] * B[k + (j * B_n)];
      }
    }
  }
}

void gl_scale(gl_float_t factor,
              gl_float_t *A,
              const int num_rows,
              const int num_cols) {
  for (int i = 0; i < (num_rows * num_cols); i++) {
    A[i] *= factor;
  }
}

gl_float_t gl_norm(const gl_float_t *x, const int size) {
  gl_float_t sum_sq = 0.0f;
  for (int i = 0; i < size; i++) {
    sum_sq += x[i] * x[i];
  }

  return sqrt(sum_sq);
}

void gl_normalize(gl_float_t *x, const int size) {
  const gl_float_t n = gl_norm(x, size);
  for (int i = 0; i < size; i++) {
    x[i] /= n;
  }
}

void gl_perspective(const gl_float_t fov,
                    const gl_float_t aspect,
                    const gl_float_t near,
                    const gl_float_t far,
                    gl_float_t P[4 * 4]) {
  const gl_float_t f = 1.0f / tan(fov * 0.5f);

  gl_zeros(P, 4, 4);
  P[0] = f / aspect;
  P[1] = 0.0f;
  P[2] = 0.0f;
  P[3] = 0.0f;

  P[4] = 0.0f;
  P[5] = f;
  P[6] = 0.0f;
  P[7] = 0.0f;

  P[8] = 0.0f;
  P[9] = 0.0f;
  P[10] = (far + near) / (near - far);
  P[11] = -1;

  P[12] = 0.0f;
  P[13] = 0.0f;
  P[14] = (2 * far * near) / (near - far);
  P[15] = 0.0f;
}

void gl_ortho(const gl_float_t left,
              const gl_float_t right,
              const gl_float_t bottom,
              const gl_float_t top,
              const gl_float_t znear,
              const gl_float_t zfar,
              gl_float_t P[4 * 4]) {
  gl_zeros(P, 4, 4);
  P[0] = 2.0f / (right - left);
  P[1] = 0.0f;
  P[2] = 0.0f;
  P[3] = 0.0f;

  P[4] = 0.0f;
  P[5] = 2.0f / (top - bottom);
  P[6] = 0.0f;
  P[7] = 0.0f;

  P[8] = 0.0f;
  P[9] = 0.0f;
  P[10] = -2.0f / (zfar - znear);
  P[11] = 0.0f;

  P[12] = -(right + left) / (right - left);
  P[13] = -(top + bottom) / (top - bottom);
  P[14] = -(zfar + znear) / (zfar - znear);
  P[15] = 1.0f;
}

void gl_lookat(const gl_float_t eye[3],
               const gl_float_t at[3],
               const gl_float_t up[3],
               gl_float_t V[4 * 4]) {
  // Z-axis: Camera forward
  gl_float_t z[3] = {0};
  gl_sub(at, eye, 3, 1, z);
  gl_normalize(z, 3);

  // X-axis: Camera right
  gl_float_t x[3] = {0};
  gl_vec3_cross(z, up, x);
  gl_normalize(x, 3);

  // Y-axis: Camera up
  gl_float_t y[3] = {0};
  gl_vec3_cross(x, z, y);

  // Negate z-axis
  gl_scale(-1.0f, z, 3, 1);

  // Form rotation component
  gl_float_t R[4 * 4] = {0};
  R[0] = x[0];
  R[1] = y[0];
  R[2] = z[0];
  R[3] = 0.0f;

  R[4] = x[1];
  R[5] = y[1];
  R[6] = z[1];
  R[7] = 0.0f;

  R[8] = x[2];
  R[9] = y[2];
  R[10] = z[2];
  R[11] = 0.0f;

  R[12] = 0.0f;
  R[13] = 0.0f;
  R[14] = 0.0f;
  R[15] = 1.0f;

  // Form translation component
  gl_float_t T[4 * 4] = {0};
  T[0] = 1.0f;
  T[1] = 0.0f;
  T[2] = 0.0f;
  T[3] = 0.0f;

  T[4] = 0.0f;
  T[5] = 1.0f;
  T[6] = 0.0f;
  T[7] = 0.0f;

  T[8] = 0.0f;
  T[9] = 0.0f;
  T[10] = 1.0f;
  T[11] = 0.0f;

  T[12] = -eye[0];
  T[13] = -eye[1];
  T[14] = -eye[2];
  T[15] = 1.0f;

  // Form view matrix
  gl_zeros(V, 4, 4);
  gl_dot(R, 4, 4, T, 4, 4, V);
}

int gl_save_frame_buffer(const int width, const int height, const char *fp) {
  // Malloc pixels
  const int num_channels = 3;
  const size_t num_pixels = num_channels * width * height;
  GLubyte *pixels = malloc(sizeof(GLubyte) * num_pixels);

  // Read pixels
  const gl_int_t x = 0;
  const gl_int_t y = 0;
  glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

  // Write to file
  GLsizei stride = num_channels * width;
  stride += (stride % 4) ? (4 - stride % 4) : 0;
  stbi_flip_vertically_on_write(1);
  stbi_write_png(fp, width, height, num_channels, pixels, stride);

  // Clean up
  if (pixels) {
    free(pixels);
  }

  return 0;
}

/******************************************************************************
 * GL-ENTITY
 *****************************************************************************/

/**
 * Convert Quaternion `q` to 3x3 rotation matrix `C`.
 */
void gl_quat2rot(const gl_float_t q[4], gl_float_t C[3 * 3]) {
  assert(q != NULL);
  assert(C != NULL);

  const gl_float_t qw = q[0];
  const gl_float_t qx = q[1];
  const gl_float_t qy = q[2];
  const gl_float_t qz = q[3];

  const gl_float_t qx2 = qx * qx;
  const gl_float_t qy2 = qy * qy;
  const gl_float_t qz2 = qz * qz;
  const gl_float_t qw2 = qw * qw;

  // Homogeneous form
  // -- 1st row
  C[0] = qw2 + qx2 - qy2 - qz2;
  C[3] = 2 * (qx * qy - qw * qz);
  C[6] = 2 * (qx * qz + qw * qy);
  // -- 2nd row
  C[1] = 2 * (qx * qy + qw * qz);
  C[4] = qw2 - qx2 + qy2 - qz2;
  C[7] = 2 * (qy * qz - qw * qx);
  // -- 3rd row
  C[2] = 2 * (qx * qz - qw * qy);
  C[5] = 2 * (qy * qz + qw * qx);
  C[8] = qw2 - qx2 - qy2 + qz2;
}

/**
 * Convert 3x3 rotation matrix `C` to Quaternion `q`.
 */
void gl_rot2quat(const gl_float_t C[3 * 3], gl_float_t q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const gl_float_t C00 = C[0];
  const gl_float_t C01 = C[3];
  const gl_float_t C02 = C[6];
  const gl_float_t C10 = C[1];
  const gl_float_t C11 = C[4];
  const gl_float_t C12 = C[7];
  const gl_float_t C20 = C[2];
  const gl_float_t C21 = C[5];
  const gl_float_t C22 = C[8];

  const gl_float_t tr = C00 + C11 + C22;
  gl_float_t S = 0.0f;
  gl_float_t qw = 0.0f;
  gl_float_t qx = 0.0f;
  gl_float_t qy = 0.0f;
  gl_float_t qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

void gl_entity_setup(gl_entity_t *entity) {
  gl_eye(entity->T, 4, 4);
  gl_eye(entity->P, 4, 4);
  entity->program_id = 0;
  entity->VAO = 0;
  entity->VBO = 0;
  entity->EBO = 0;
}

void gl_entity_cleanup(gl_entity_t *entity) {
  if (glIsProgram(entity->program_id) == GL_TRUE) {
    glDeleteProgram(entity->program_id);
  }

  if (glIsVertexArray(entity->VAO) == GL_TRUE) {
    glDeleteVertexArrays(1, &entity->VAO);
  }

  if (glIsBuffer(entity->VBO) == GL_TRUE) {
    glDeleteBuffers(1, &entity->VBO);
  }

  if (glIsBuffer(entity->EBO) == GL_TRUE) {
    glDeleteBuffers(1, &entity->EBO);
  }
}

void gl_entity_set_position(gl_entity_t *entity, const gl_float_t pos[3]) {
  assert(entity != NULL);
  assert(pos != NULL);
  entity->T[12] = pos[0];
  entity->T[13] = pos[1];
  entity->T[14] = pos[2];
}

void gl_entity_get_position(gl_entity_t *entity, gl_float_t *pos) {
  assert(entity != NULL);
  assert(pos != NULL);
  pos[0] = entity->T[12];
  pos[1] = entity->T[13];
  pos[2] = entity->T[14];
}

void gl_entity_set_rotation(gl_entity_t *entity, const gl_float_t rot[3 * 3]) {
  assert(entity != NULL);
  assert(rot != NULL);
  entity->T[0] = rot[0];
  entity->T[1] = rot[1];
  entity->T[2] = rot[2];
  entity->T[4] = rot[3];
  entity->T[5] = rot[4];
  entity->T[6] = rot[5];
  entity->T[8] = rot[6];
  entity->T[9] = rot[7];
  entity->T[10] = rot[8];
}

void gl_entity_get_rotation(gl_entity_t *entity, gl_float_t *rot) {
  assert(entity != NULL);
  assert(rot != NULL);

  rot[0] = entity->T[0];
  rot[1] = entity->T[1];
  rot[2] = entity->T[2];
  rot[3] = entity->T[4];
  rot[4] = entity->T[5];
  rot[5] = entity->T[6];
  rot[6] = entity->T[8];
  rot[7] = entity->T[9];
  rot[8] = entity->T[10];
}

void gl_entity_set_quaternion(gl_entity_t *entity, const gl_float_t quat[4]) {
  assert(entity != NULL);
  assert(quat != NULL);
  gl_float_t C[3 * 3] = {0};
  gl_quat2rot(quat, C);
  gl_entity_set_rotation(entity, C);
}

void gl_entity_get_quaternion(gl_entity_t *entity, gl_float_t *quat) {
  assert(entity != NULL);
  assert(quat != NULL);
  gl_float_t C[3 * 3] = {0};
  gl_entity_get_rotation(entity, C);
  gl_rot2quat(C, quat);
}

/******************************************************************************
 * GL-SHADER
 *****************************************************************************/

gl_uint_t gl_shader_compile(const char *shader_src, const int type) {
  if (shader_src == NULL) {
    LOG_ERROR("Shader source is NULL!");
    return GL_FALSE;
  }

  const gl_uint_t shader = glCreateShader(type);
  glShaderSource(shader, 1, &shader_src, NULL);
  glCompileShader(shader);

  gl_int_t retval = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &retval);
  if (retval == GL_FALSE) {
    char log[9046] = {0};
    glGetShaderInfoLog(shader, 9046, NULL, log);
    LOG_ERROR("Failed to compile shader:\n%s", log);
    return retval;
  }

  return shader;
}

void gl_shader_status(gl_uint_t shader) {
  gl_int_t success = 0;
  GLchar infoLog[1024];

  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shader, 1024, NULL, infoLog);
    printf("ERROR::SHADER_COMPILATION_ERROR:\n");
    printf("%s\n", infoLog);
    printf("\n -- --------------------------------------------------- -- ");
    printf("\n");
  }

  glGetProgramiv(shader, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shader, 1024, NULL, infoLog);
    printf("ERROR::PROGRAM_LINKING_ERROR:\n");
    printf("%s\n", infoLog);
    printf("\n -- --------------------------------------------------- -- ");
    printf("\n");
  }
}

gl_uint_t gl_shaders_link(const gl_uint_t vertex_shader,
                          const gl_uint_t fragment_shader,
                          const gl_uint_t geometry_shader) {
  // Attach shaders to link
  gl_uint_t program = glCreateProgram();
  glAttachShader(program, vertex_shader);
  glAttachShader(program, fragment_shader);
  if (geometry_shader != GL_FALSE) {
    glAttachShader(program, geometry_shader);
  }
  glLinkProgram(program);

  // Link program
  gl_int_t success = 0;
  char log[9046];
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (success == GL_FALSE) {
    glGetProgramInfoLog(program, 9046, NULL, log);
    LOG_ERROR("Failed to link shaders:\nReason: %s\n", log);
    return GL_FALSE;
  }

  // Delete shaders
  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);
  if (geometry_shader == GL_FALSE) {
    glDeleteShader(geometry_shader);
  }

  return program;
}

/******************************************************************************
 * GL-PROGRAM
 *****************************************************************************/

gl_uint_t gl_prog_setup(const char *vs_src,
                        const char *fs_src,
                        const char *gs_src) {
  gl_uint_t vs = GL_FALSE;
  gl_uint_t fs = GL_FALSE;
  gl_uint_t gs = GL_FALSE;

  if (vs_src) {
    vs = gl_shader_compile(vs_src, GL_VERTEX_SHADER);
  }

  if (fs_src) {
    fs = gl_shader_compile(fs_src, GL_FRAGMENT_SHADER);
  }

  if (gs_src) {
    gs = gl_shader_compile(gs_src, GL_GEOMETRY_SHADER);
  }

  const gl_uint_t program_id = gl_shaders_link(vs, fs, gs);
  return program_id;
}

int gl_prog_set_int(const gl_int_t id, const char *k, const gl_int_t v) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1i(location, v);
  return 0;
}

int gl_prog_set_vec2i(const gl_int_t id, const char *k, const gl_int_t v[2]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2i(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3i(const gl_int_t id, const char *k, const gl_int_t v[3]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3i(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4i(const gl_int_t id, const char *k, const gl_int_t v[4]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4i(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_color(const gl_int_t id, const char *k, const gl_color_t v) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3f(location, v.r, v.g, v.b);
  return 0;
}

int gl_prog_set_float(const gl_int_t id, const char *k, const gl_float_t v) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1f(location, v);
  return 0;
}

int gl_prog_set_vec2(const gl_int_t id, const char *k, const gl_float_t v[2]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2f(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3(const gl_int_t id, const char *k, const gl_float_t v[3]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3f(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4(const gl_int_t id, const char *k, const gl_float_t v[4]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4f(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_mat2(const gl_int_t id,
                     const char *k,
                     const gl_float_t v[2 * 2]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix2fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat3(const gl_int_t id,
                     const char *k,
                     const gl_float_t v[3 * 3]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix3fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat4(const gl_int_t id,
                     const char *k,
                     const gl_float_t v[4 * 4]) {
  const gl_int_t location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix4fv(location, 1, GL_FALSE, v);
  return 0;
}

/******************************************************************************
 * GL-CAMERA
 *****************************************************************************/

void gl_camera_setup(gl_camera_t *camera,
                     int *window_width,
                     int *window_height) {
  camera->view_mode = FPS;
  camera->window_width = window_width;
  camera->window_height = window_height;

  gl_zeros(camera->focal, 3, 1);
  gl_vec3(camera->world_up, 0.0f, 1.0f, 0.0f);
  gl_vec3(camera->position, 0.0f, 3.0f, 0.0f);
  gl_vec3(camera->right, -1.0f, 0.0f, 0.0f);
  gl_vec3(camera->up, 0.0f, 1.0f, 0.0f);
  gl_vec3(camera->front, 0.0f, 0.0f, -1.0f);
  camera->yaw = gl_deg2rad(0.0f);
  camera->pitch = gl_deg2rad(-50.0f);
  camera->radius = 1.0f;

  camera->fov = gl_deg2rad(60.0f);
  camera->fov_min = gl_deg2rad(10.0f);
  camera->fov_max = gl_deg2rad(120.0f);
  camera->near = 0.01f;
  camera->far = 100.0f;

  gl_camera_update(camera);
}

void gl_camera_update(gl_camera_t *camera) {
  // Front vector
  camera->front[0] = sin(camera->yaw) * cos(camera->pitch);
  camera->front[1] = sin(camera->pitch);
  camera->front[2] = cos(camera->yaw) * cos(camera->pitch);
  gl_normalize(camera->front, 3);

  // Right vector
  gl_vec3_cross(camera->front, camera->world_up, camera->right);
  gl_normalize(camera->right, 3);

  // Up vector
  gl_vec3_cross(camera->right, camera->front, camera->up);
  gl_normalize(camera->up, 3);

  // Projection matrix
  const float width = (float) *(camera->window_width);
  const float height = (float) *(camera->window_height);
  const float aspect = width / height;
  gl_perspective(camera->fov, aspect, camera->near, camera->far, camera->P);

  // View matrix (Orbit mode)
  if (camera->view_mode == ORBIT) {
    camera->position[0] =
        camera->radius * sin(camera->pitch) * sin(camera->yaw);
    camera->position[1] = camera->radius * cos(camera->pitch);
    camera->position[2] =
        camera->radius * sin(camera->pitch) * cos(camera->yaw);

    gl_float_t eye[3] = {0};
    eye[0] = camera->position[0];
    eye[1] = camera->position[1];
    eye[2] = camera->position[2];
    gl_lookat(eye, camera->focal, camera->world_up, camera->V);
  }

  // View matrix (FPS mode)
  if (camera->view_mode == FPS) {
    gl_float_t eye[3] = {0};
    eye[0] = camera->position[0];
    eye[1] = camera->position[1];
    eye[2] = camera->position[2];

    gl_float_t carrot[3] = {0};
    carrot[0] = camera->position[0] + camera->front[0];
    carrot[1] = camera->position[1] + camera->front[1];
    carrot[2] = camera->position[2] + camera->front[2];

    gl_lookat(eye, carrot, camera->world_up, camera->V);
  }
}

void gl_camera_rotate(gl_camera_t *camera,
                      const float factor,
                      const float dx,
                      const float dy) {
  // Update yaw and pitch
  float pitch = camera->pitch;
  float yaw = camera->yaw;
  yaw -= dx * factor;
  pitch -= dy * factor;

  // Constrain pitch and yaw
  pitch = (pitch > gl_deg2rad(89.99f)) ? gl_deg2rad(89.99f) : pitch;
  pitch = (pitch < gl_deg2rad(-89.99f)) ? gl_deg2rad(-89.99f) : pitch;

  // Update camera attitude
  camera->pitch = pitch;
  camera->yaw = yaw;

  // Update camera forward
  float direction[3] = {0};
  direction[0] = cos(yaw) * cos(pitch);
  direction[1] = sin(pitch);
  direction[2] = sin(yaw) * cos(pitch);
  gl_normalize(direction, 3);

  camera->front[0] = direction[0];
  camera->front[1] = direction[1];
  camera->front[2] = direction[2];
}

void gl_camera_pan(gl_camera_t *camera,
                   const float factor,
                   const float dx,
                   const float dy) {
  // camera->focal -= (dy * mouse_sensitivity) * camera->front;
  // camera->focal += (dx * mouse_sensitivity) * camera->right;
  const gl_float_t dx_scaled = dx * factor;
  const gl_float_t dy_scaled = dy * factor;
  gl_float_t front[3] = {camera->front[0], camera->front[1], camera->front[2]};
  gl_float_t right[3] = {camera->right[0], camera->right[1], camera->right[2]};
  gl_scale(dy_scaled, front, 3, 1);
  gl_scale(dx_scaled, right, 3, 1);
  gl_sub(camera->focal, front, 3, 1, camera->focal);
  gl_add(camera->focal, right, 3, 1, camera->focal);

  // limit focal point y-axis
  camera->focal[1] = (camera->focal[1] < 0) ? 0 : camera->focal[1];
}

void gl_camera_zoom(gl_camera_t *camera,
                    const float factor,
                    const float dx,
                    const float dy) {
  UNUSED(factor);
  UNUSED(dx);
  gl_float_t fov = camera->fov + dy;
  fov = (fov <= camera->fov_min) ? camera->fov_min : fov;
  fov = (fov >= camera->fov_max) ? camera->fov_max : fov;
  camera->fov = fov;
}

/******************************************************************************
 * GL-PRIMITIVES
 *****************************************************************************/

// GL RECT ///////////////////////////////////////////////////////////////////

#define GL_RECT_VS                                                             \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec2 in_pos;\n"                                    \
  "layout (location = 1) in vec2 in_tex_coord;\n"                              \
  "out vec2 tex_coord;\n"                                                      \
  "void main() {\n"                                                            \
  "  gl_Position = vec4(in_pos, 0.0f, 1.0f);\n"                                \
  "  tex_coord = in_tex_coord;\n"                                              \
  "}\n"

#define GL_RECT_FS                                                             \
  "#version 330 core\n"                                                        \
  "out vec4 frag_color;\n"                                                     \
  "void main() {\n"                                                            \
  "  frag_color = vec4(0.5f, 0.5f, 01.0f, 1.0f);\n"                            \
  "}\n"

gl_rect_t *gl_rect_malloc(const gl_bounds_t bounds) {
  // Malloc
  gl_rect_t *rect = MALLOC(gl_rect_t, 1);
  gl_entity_setup(&rect->entity);
  rect->bounds = bounds;

  // Shader program
  rect->entity.program_id = gl_prog_setup(GL_RECT_VS, GL_RECT_FS, NULL);
  if (rect->entity.program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Vertices
  // clang-format off
  const float vertices[4 * 2] = {
     -0.5f, -0.5f, // Bottom left
     0.5f, -0.5f,  // Bottom right
     0.5f,  0.5f,  // Top right
     -0.5f,  0.5f, // Top left
  };
  // clang-format on

  // VAO
  glGenVertexArrays(1, &rect->entity.VAO);
  glBindVertexArray(rect->entity.VAO);

  // VBO
  glGenBuffers(1, &rect->entity.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, rect->entity.VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t pos_size = sizeof(float) * 2;
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, pos_size, pos_offset);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO

  return rect;
}

void gl_rect_free(gl_rect_t *rect) {
  if (rect == NULL) {
    return;
  }
  gl_entity_cleanup(&rect->entity);
  free(rect);
}

void gl_rect_draw(const gl_rect_t *rect, const gl_camera_t *camera) {
  glUseProgram(rect->entity.program_id);
  glBindVertexArray(rect->entity.VAO);
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  glBindVertexArray(0);
}

// GL CUBE ///////////////////////////////////////////////////////////////////

#define GL_CUBE_VS                                                             \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec3 in_pos;\n"                                    \
  "out vec3 color;\n"                                                          \
  "uniform mat4 model;\n"                                                      \
  "uniform mat4 view;\n"                                                       \
  "uniform mat4 projection;\n"                                                 \
  "uniform vec3 in_color;\n"                                                   \
  "void main() {\n"                                                            \
  "  gl_Position = projection * view * model * vec4(in_pos, 1.0);\n"           \
  "  color = in_color;\n"                                                      \
  "}\n"

#define GL_CUBE_FS                                                             \
  "#version 330 core\n"                                                        \
  "in vec3 color;\n"                                                           \
  "out vec4 frag_color;\n"                                                     \
  "void main() {\n"                                                            \
  "  frag_color = vec4(color, 1.0f);\n"                                        \
  "}\n"

gl_cube_t *gl_cube_malloc(const gl_float_t size,
                          const gl_float_t pos[3],
                          const gl_float_t color[3],
                          const gl_float_t outline_color[3]) {
  // Malloc
  gl_cube_t *cube = MALLOC(gl_cube_t, 1);
  gl_entity_setup(&cube->entity);

  // Transform
  cube->entity.T[12] = pos[0];
  cube->entity.T[13] = pos[1];
  cube->entity.T[14] = pos[2];

  // Size
  cube->size = size;

  // Color
  cube->color[0] = color[0];
  cube->color[1] = color[1];
  cube->color[2] = color[2];

  // Outline color
  cube->outline_color[0] = outline_color[0];
  cube->outline_color[1] = outline_color[1];
  cube->outline_color[2] = outline_color[2];

  // Shader program
  cube->entity.program_id = gl_prog_setup(GL_CUBE_VS, GL_CUBE_FS, NULL);
  if (cube->entity.program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Vertices
  // clang-format off
  gl_float_t vertices[12 * 3 * 3] = {
    // Triangle 1
    -size, -size, -size,
    -size, -size,  size,
    -size,  size,  size,
    // Triangle 2
     size,  size, -size,
    -size, -size, -size,
    -size,  size, -size,
    // Triangle 3
     size, -size,  size,
    -size, -size, -size,
     size, -size, -size,
    // Triangle 4
     size,  size, -size,
     size, -size, -size,
    -size, -size, -size,
    // Triangle 5
    -size, -size, -size,
    -size,  size,  size,
    -size,  size, -size,
    // Triangle 6
     size, -size,  size,
    -size, -size,  size,
    -size, -size, -size,
    // Triangle 7
    -size,  size,  size,
    -size, -size,  size,
     size, -size,  size,
    // Triangle 8
     size,  size,  size,
     size, -size, -size,
     size,  size, -size,
    // Triangle 9
     size, -size, -size,
     size,  size,  size,
     size, -size,  size,
    // Triangle 10
     size,  size,  size,
     size,  size, -size,
    -size,  size, -size,
    // Triangle 11
     size,  size,  size,
    -size,  size, -size,
    -size,  size,  size,
    // Triangle 12
     size,  size,  size,
    -size,  size,  size,
     size, -size,  size
    // Triangle 12 : end
  };
  const size_t num_triangles = 12;
  const size_t vertices_per_triangle = 3;
  const size_t num_vertices = vertices_per_triangle * num_triangles;
  const size_t vbo_size = sizeof(gl_float_t) * num_vertices * 3;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &cube->entity.VAO);
  glBindVertexArray(cube->entity.VAO);

  // VBO
  glGenBuffers(1, &cube->entity.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, cube->entity.VBO);
  glBufferData(GL_ARRAY_BUFFER, vbo_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  const size_t vertex_size = sizeof(gl_float_t) * 3;
  const void *offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, offset);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO

  return cube;
}

void gl_cube_free(gl_cube_t *cube) {
  if (cube == NULL) {
    return;
  }
  gl_entity_cleanup(&cube->entity);
  free(cube);
}

void gl_cube_draw(const gl_cube_t *cube, const gl_camera_t *camera) {
  glUseProgram(cube->entity.program_id);

  // Draw cube
  gl_prog_set_mat4(cube->entity.program_id, "projection", camera->P);
  gl_prog_set_mat4(cube->entity.program_id, "view", camera->V);
  gl_prog_set_mat4(cube->entity.program_id, "model", cube->entity.T);
  gl_prog_set_vec3(cube->entity.program_id, "in_color", cube->color);
  glBindVertexArray(cube->entity.VAO);
  glDrawArrays(GL_TRIANGLES, 0, 36); // 36 Vertices

  // // Draw outline
  // // -- Track original line width
  // gl_float_t lw_orig;
  // glGetFloatv(GL_LINE_WIDTH, &lw_orig);
  // // -- Draw Outline
  // gl_float_t outline[4 * 4] = {0};
  // gl_copy(cube->T, 4, 4, outline);
  // gl_prog_set_mat4(cube->entity.program_id, "model", outline);
  // gl_prog_set_vec3(cube->entity.program_id, "in_color", cube->outline_color);
  // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Wireframe mode
  // glLineWidth(2.0f);
  // glDrawArrays(GL_TRIANGLES, 0, 36);
  // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Reset to fill mode
  // // -- Restore original line width
  // glLineWidth(lw_orig);

  // Unbind VAO
  glBindVertexArray(0);
}

// GL CAMERA FRAME ///////////////////////////////////////////////////////////

#define GL_CFRAME_VS                                                           \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec3 in_pos;\n"                                    \
  "uniform mat4 model;\n"                                                      \
  "uniform mat4 view;\n"                                                       \
  "uniform mat4 projection;\n"                                                 \
  "void main() {\n"                                                            \
  "  gl_Position = projection * view * model * vec4(in_pos, 1.0);\n"           \
  "}\n"

#define GL_CFRAME_FS                                                           \
  "#version 150 core\n"                                                        \
  "out vec4 frag_color;\n"                                                     \
  "void main() {\n"                                                            \
  "  frag_color = vec4(1.0f, 1.0f, 1.0f, 1.0f);\n"                             \
  "}\n"

gl_cframe_t *gl_cframe_malloc(const gl_float_t pos[3],
                              const gl_float_t size,
                              const gl_float_t color[3]) {
  // Malloc
  gl_cframe_t *cf = MALLOC(gl_cframe_t, 1);
  gl_entity_setup(&cf->entity);
  cf->size = size;
  gl_copy(color, 3, 1, cf->color);
  // cf->line_width = line_width;

  cf->entity.T[12] = pos[0];
  cf->entity.T[13] = pos[1];
  cf->entity.T[14] = pos[2];

  // Shader program
  cf->entity.program_id = gl_prog_setup(GL_CFRAME_VS, GL_CFRAME_FS, NULL);
  if (cf->entity.program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Form the camera fov frame
  gl_float_t fov = gl_deg2rad(60.0);
  gl_float_t hfov = fov / 2.0f;
  gl_float_t scale = 1.0f;
  gl_float_t z = scale;
  gl_float_t hwidth = z * tan(hfov);
  const gl_float_t lb[3] = {-hwidth, hwidth, z};  // Left bottom
  const gl_float_t lt[3] = {-hwidth, -hwidth, z}; // Left top
  const gl_float_t rt[3] = {hwidth, -hwidth, z};  // Right top
  const gl_float_t rb[3] = {hwidth, hwidth, z};   // Right bottom

  // Rectangle frame
  // clang-format off
  const size_t vertex_size = sizeof(gl_float_t) * 3;
  const gl_float_t vertices[8 * 6] = {
    // -- Left bottom to left top
    lb[0], lb[1], lb[2], lt[0], lt[1], lt[2],
    // -- Left top to right top
    lt[0], lt[1], lt[2], rt[0], rt[1], rt[2],
    // -- Right top to right bottom
    rt[0], rt[1], rt[2], rb[0], rb[1], rb[2],
    // -- Right bottom to left bottom
    rb[0], rb[1], rb[2], lb[0], lb[1], lb[2],
    // Rectangle frame to origin
    // -- Origin to left bottom
    0.0f, 0.0f, 0.0f, lb[0], lb[1], lb[2],
    // -- Origin to left top
    0.0f, 0.0f, 0.0f, lt[0], lt[1], lt[2],
    // -- Origin to right top
    0.0f, 0.0f, 0.0f, rt[0], rt[1], rt[2],
    // -- Origin to right bottom
    0.0f, 0.0f, 0.0f, rb[0], rb[1], rb[2]
  };
  // clang-format on

  // VAO
  glGenVertexArrays(1, &cf->entity.VAO);
  glBindVertexArray(cf->entity.VAO);

  // VBO
  glGenBuffers(1, &cf->entity.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, cf->entity.VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO

  return cf;
}

void gl_cframe_free(gl_cframe_t *cf) {
  if (cf == NULL) {
    return;
  }
  gl_entity_cleanup(&cf->entity);
  free(cf);
}

void gl_cframe_draw(const gl_cframe_t *cf, const gl_camera_t *camera) {
  glUseProgram(cf->entity.program_id);
  gl_prog_set_mat4(cf->entity.program_id, "projection", camera->P);
  gl_prog_set_mat4(cf->entity.program_id, "view", camera->V);
  gl_prog_set_mat4(cf->entity.program_id, "model", cf->entity.T);

  // Store original line width
  gl_float_t original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  gl_float_t line_width = 2.0f;
  glLineWidth(line_width);

  // Draw frame
  const size_t num_lines = 8;
  const size_t num_vertices = num_lines * 2;
  glBindVertexArray(cf->entity.VAO);
  glDrawArrays(GL_LINES, 0, num_vertices);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

// GL AXIS FRAME /////////////////////////////////////////////////////////////

#define GL_AXES3D_VS                                                           \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec3 in_pos;\n"                                    \
  "layout (location = 1) in vec3 in_color;\n"                                  \
  "out vec3 color;\n"                                                          \
  "uniform mat4 model;\n"                                                      \
  "uniform mat4 view;\n"                                                       \
  "uniform mat4 projection;\n"                                                 \
  "void main() {\n"                                                            \
  "  gl_Position = projection * view * model * vec4(in_pos, 1.0);\n"           \
  "  color = in_color;\n"                                                      \
  "}\n"

#define GL_AXES3D_FS                                                           \
  "#version 150 core\n"                                                        \
  "in vec3 color;\n"                                                           \
  "out vec4 frag_color;\n"                                                     \
  "void main() {\n"                                                            \
  "  frag_color = vec4(color, 1.0f);\n"                                        \
  "}\n"

gl_axes3d_t *gl_axes3d_malloc(void) {
  // Malloc
  gl_axes3d_t *axes = MALLOC(gl_axes3d_t, 1);
  gl_entity_setup(&axes->entity);
  axes->size = 1.0f;
  axes->line_width = 6.0f;

  // Shader program
  axes->entity.program_id = gl_prog_setup(GL_AXES3D_VS, GL_AXES3D_FS, NULL);
  if (axes->entity.program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Vertices
  // clang-format off
  static const gl_float_t vertices[] = {
    // Line 1 : x-axis + color
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    // Line 2 : y-axis + color
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    // Line 3 : z-axis + color
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f
  };
  // clang-format on

  // VAO
  glGenVertexArrays(1, &axes->entity.VAO);
  glBindVertexArray(axes->entity.VAO);

  // VBO
  glGenBuffers(1, &axes->entity.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, axes->entity.VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO

  return axes;
}

void gl_axes3d_free(gl_axes3d_t *axes) {
  if (axes == NULL) {
    return;
  }
  gl_entity_cleanup(&axes->entity);
  free(axes);
}

void gl_axes3d_draw(const gl_axes3d_t *axes, const gl_camera_t *camera) {
  glUseProgram(axes->entity.program_id);
  gl_prog_set_mat4(axes->entity.program_id, "projection", camera->P);
  gl_prog_set_mat4(axes->entity.program_id, "view", camera->V);
  gl_prog_set_mat4(axes->entity.program_id, "model", axes->entity.T);

  // Store original line width
  gl_float_t original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  glLineWidth(axes->line_width);

  // Draw frame
  glBindVertexArray(axes->entity.VAO);
  glDrawArrays(GL_LINES, 0, 6);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

// GL GRID ///////////////////////////////////////////////////////////////////

#define GL_GRID3D_VS                                                           \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec3 in_pos;\n"                                    \
  "uniform mat4 model;\n"                                                      \
  "uniform mat4 view;\n"                                                       \
  "uniform mat4 projection;\n"                                                 \
  "void main() {\n"                                                            \
  "  gl_Position = projection * view * model * vec4(in_pos, 1.0);\n"           \
  "}\n"

#define GL_GRID3D_FS                                                           \
  "#version 150 core\n"                                                        \
  "out vec4 frag_color;\n"                                                     \
  "void main() {\n"                                                            \
  "  frag_color = vec4(0.8f, 0.8f, 0.8f, 1.0f);\n"                             \
  "}\n"

static gl_float_t *gl_grid3d_create_vertices(int grid_size) {
  // Allocate memory for vertices
  const int num_lines = (grid_size + 1) * 2;
  const int num_vertices = num_lines * 2;
  const size_t buffer_size = sizeof(gl_float_t) * num_vertices * 3;
  gl_float_t *vertices = (gl_float_t *) malloc(buffer_size);

  // Setup
  const float min_x = -1.0f * (float) grid_size / 2.0f;
  const float max_x = (float) grid_size / 2.0f;
  const float min_z = -1.0f * (float) grid_size / 2.0f;
  const float max_z = (float) grid_size / 2.0f;

  // Row vertices
  float z = min_z;
  int vert_idx = 0;
  for (int i = 0; i < ((grid_size + 1) * 2); i++) {
    if ((i % 2) == 0) {
      vertices[(vert_idx * 3)] = min_x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = z;
    } else {
      vertices[(vert_idx * 3)] = max_x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = z;
      z += 1.0f;
    }
    vert_idx++;
  }

  // Column vertices
  float x = min_x;
  for (int j = 0; j < ((grid_size + 1) * 2); j++) {
    if ((j % 2) == 0) {
      vertices[(vert_idx * 3)] = x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = min_z;
    } else {
      vertices[(vert_idx * 3)] = x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = max_z;
      x += 1.0f;
    }
    vert_idx++;
  }

  return vertices;
}

gl_grid3d_t *gl_grid3d_malloc(void) {
  // Malloc
  gl_grid3d_t *grid = MALLOC(gl_grid3d_t, 1);
  gl_entity_setup(&grid->entity);
  grid->size = 1.0f;
  grid->line_width = 2.0f;

  // Shader program
  grid->entity.program_id = gl_prog_setup(GL_GRID3D_VS, GL_GRID3D_FS, NULL);
  if (grid->entity.program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Create vertices
  const int grid_size = 10;
  const size_t vertex_size = sizeof(gl_float_t) * 3;
  const void *offset = (void *) 0;
  const int num_lines = (grid_size + 1) * 2;
  const int num_vertices = num_lines * 2;
  gl_float_t *vertices = gl_grid3d_create_vertices(grid_size);
  const size_t buffer_size = sizeof(gl_float_t) * num_vertices * 3;

  // VAO
  glGenVertexArrays(1, &grid->entity.VAO);
  glBindVertexArray(grid->entity.VAO);

  // VBO
  glGenBuffers(1, &grid->entity.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, grid->entity.VBO);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, offset);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
  free(vertices);

  return grid;
}

void gl_grid3d_free(gl_grid3d_t *grid) {
  if (grid == NULL) {
    return;
  }
  gl_entity_cleanup(&grid->entity);
  free(grid);
}

void gl_grid3d_draw(const gl_grid3d_t *grid, const gl_camera_t *camera) {
  glUseProgram(grid->entity.program_id);
  gl_prog_set_mat4(grid->entity.program_id, "projection", camera->P);
  gl_prog_set_mat4(grid->entity.program_id, "view", camera->V);
  gl_prog_set_mat4(grid->entity.program_id, "model", grid->entity.T);

  const int grid_size = 10;
  const int num_lines = (grid_size + 1) * 2;
  const int num_vertices = num_lines * 2;

  glBindVertexArray(grid->entity.VAO);
  glDrawArrays(GL_LINES, 0, num_vertices);
  glBindVertexArray(0); // Unbind VAO
}

// GL POINTS /////////////////////////////////////////////////////////////////

#define GL_POINTS3D_VS                                                         \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec3 in_pos;\n"                                    \
  "layout (location = 1) in vec3 in_color;\n"                                  \
  "out vec3 color;\n"                                                          \
  "uniform mat4 model;\n"                                                      \
  "uniform mat4 view;\n"                                                       \
  "uniform mat4 projection;\n"                                                 \
  "void main() {\n"                                                            \
  "  gl_Position = projection * view * model * vec4(in_pos, 1.0);\n"           \
  "  gl_PointSize = 1.0;\n"                                                    \
  "  color = in_color;\n"                                                      \
  "}\n"

#define GL_POINTS3D_FS                                                         \
  "#version 330 core\n"                                                        \
  "in vec3 color;\n"                                                           \
  "out vec4 frag_color;\n"                                                     \
  "void main() {\n"                                                            \
  "  frag_color = vec4(color, 1.0f);\n"                                        \
  "}\n"

gl_points3d_t *gl_points3d_malloc(const float *data,
                                  const size_t num_points,
                                  const gl_float_t size,
                                  const gl_float_t color[3]) {
  // Malloc
  gl_points3d_t *points = MALLOC(gl_points3d_t, 1);
  gl_entity_setup(&points->entity);
  points->data = MALLOC(gl_float_t, num_points * 3);
  for (size_t i = 0; i < num_points * 3; ++i) {
    points->data[i] = data[i];
  }
  points->num_points = num_points;
  points->size = size;
  gl_copy(color, 3, 1, points->color);

  // Shader program
  points->entity.program_id =
      gl_prog_setup(GL_POINTS3D_VS, GL_POINTS3D_FS, NULL);
  if (points->entity.program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw points!");
  }

  // Points
  const float r = points->color[0];
  const float g = points->color[1];
  const float b = points->color[2];
  gl_float_t *vertex_data = MALLOC(gl_float_t, num_points * 6);
  for (size_t i = 0; i < num_points; ++i) {
    vertex_data[i * 6 + 0] = data[i * 3 + 0];
    vertex_data[i * 6 + 1] = data[i * 3 + 1];
    vertex_data[i * 6 + 2] = data[i * 3 + 2];
    vertex_data[i * 6 + 3] = r;
    vertex_data[i * 6 + 4] = g;
    vertex_data[i * 6 + 5] = b;
  }
  const size_t vertex_buffer_size = sizeof(float) * 6 * num_points;

  // VAO
  glGenVertexArrays(1, &points->entity.VAO);
  glBindVertexArray(points->entity.VAO);

  // VBO
  glGenBuffers(1, &points->entity.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, points->entity.VBO);
  glBufferData(GL_ARRAY_BUFFER,
               vertex_buffer_size,
               vertex_data,
               GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
  free(vertex_data);

  return points;
}

void gl_points3d_free(gl_points3d_t *points) {
  if (points == NULL) {
    return;
  }
  gl_entity_cleanup(&points->entity);
  free(points->data);
  free(points);
}

void gl_points3d_draw(const gl_points3d_t *points,
                      const gl_camera_t *camera,
                      const size_t num_points) {
  glUseProgram(points->entity.program_id);
  gl_prog_set_mat4(points->entity.program_id, "projection", camera->P);
  gl_prog_set_mat4(points->entity.program_id, "view", camera->V);
  gl_prog_set_mat4(points->entity.program_id, "model", points->entity.T);

  glBindVertexArray(points->entity.VAO);
  glDrawArrays(GL_POINTS, 0, num_points);
  glBindVertexArray(0); // Unbind VAO
}

// GL LINE ///////////////////////////////////////////////////////////////////

#define GL_LINE3D_VS                                                           \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec3 in_pos;\n"                                    \
  "out vec3 color;\n"                                                          \
  "uniform mat4 model;\n"                                                      \
  "uniform mat4 view;\n"                                                       \
  "uniform mat4 projection;\n"                                                 \
  "uniform vec3 in_color;\n"                                                   \
  "void main() {\n"                                                            \
  "  gl_Position = projection * view * model * vec4(in_pos, 1.0);\n"           \
  "  color = in_color;\n"                                                      \
  "}\n"

#define GL_LINE3D_FS                                                           \
  "#version 330 core\n"                                                        \
  "in vec3 color;\n"                                                           \
  "out vec4 frag_color;\n"                                                     \
  "void main() {\n"                                                            \
  "  frag_color = vec4(color, 1.0f);\n"                                        \
  "}\n"

gl_line3d_t *gl_line3d_malloc(const gl_float_t *data,
                              const size_t num_points,
                              const gl_float_t line_width,
                              const gl_float_t color[3]) {
  // Malloc
  gl_line3d_t *line = MALLOC(gl_line3d_t, 1);
  gl_entity_setup(&line->entity);
  line->data = MALLOC(gl_float_t, num_points * 3);
  for (size_t i = 0; i < num_points * 3; ++i) {
    line->data[i] = data[i];
  }
  line->num_points = num_points;
  line->line_width = line_width;
  gl_copy(color, 3, 1, line->color);

  // Shader program
  line->entity.program_id = gl_prog_setup(GL_LINE3D_VS, GL_LINE3D_FS, NULL);
  if (line->entity.program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // VAO
  glGenVertexArrays(1, &line->entity.VAO);
  glBindVertexArray(line->entity.VAO);

  // VBO
  const size_t vbo_size = sizeof(gl_float_t) * 3 * num_points;
  glGenBuffers(1, &line->entity.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, line->entity.VBO);
  glBufferData(GL_ARRAY_BUFFER, vbo_size, data, GL_STATIC_DRAW);

  // Position attribute
  const size_t vertex_size = sizeof(gl_float_t) * 3;
  const void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO

  return line;
}

void gl_line3d_free(gl_line3d_t *line) {
  if (line == NULL) {
    return;
  }
  gl_entity_cleanup(&line->entity);
  free(line->data);
  free(line);
}

void gl_line3d_draw(const gl_line3d_t *line, const gl_camera_t *camera) {
  // Activate shader
  glUseProgram(line->entity.program_id);
  gl_prog_set_mat4(line->entity.program_id, "projection", camera->P);
  gl_prog_set_mat4(line->entity.program_id, "view", camera->V);
  gl_prog_set_mat4(line->entity.program_id, "model", line->entity.T);
  gl_prog_set_vec3(line->entity.program_id, "in_color", line->color);

  // Store original line width
  gl_float_t original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  glLineWidth(line->line_width);

  // Draw frame
  glBindVertexArray(line->entity.VAO);
  glDrawArrays(GL_LINE_STRIP, 0, line->num_points);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

// GL TEXT ///////////////////////////////////////////////////////////////////

#define GL_TEXT_VS                                                             \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec4 vertex;\n"                                    \
  "out vec2 tex_coords;\n"                                                     \
  "uniform mat4 projection;\n"                                                 \
  "void main() {\n"                                                            \
  "  gl_Position = projection * vec4(vertex.xy, 0.01, 1.0);\n"                 \
  "  tex_coords = vertex.zw;\n"                                                \
  "}\n"

#define GL_TEXT_FS                                                             \
  "#version 330 core\n"                                                        \
  "in vec2 tex_coords;\n"                                                      \
  "out vec4 frag_color;\n"                                                     \
  "uniform sampler2D text;\n"                                                  \
  "uniform vec3 text_color;\n"                                                 \
  "void main() {\n"                                                            \
  "  float alpha = texture(text, tex_coords).r;\n"                             \
  "  frag_color = vec4(text_color, alpha);\n"                                  \
  "}\n"

void gl_char_print(const gl_char_t *ch) {
  printf("texture_id: %d\n", ch->texture_id);
  printf("width:      %d\n", ch->size[0]);
  printf("height:     %d\n", ch->size[1]);
  printf("bearing_x:  %d\n", ch->bearing[0]);
  printf("bearing_y:  %d\n", ch->bearing[1]);
  printf("offset:     %d\n", ch->offset);
  printf("\n");
}

void gl_text_setup(gl_text_t *text) {
  // Initialize
  gl_entity_setup(&text->entity);
  text->color = (gl_color_t){0.0, 0.0, 0.0};
  text->size = 24;

  // Compile shader
  text->entity.program_id = gl_prog_setup(GL_TEXT_VS, GL_TEXT_FS, NULL);
  if (text->entity.program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Initialize FreeType library
  FT_Library ft;
  if (FT_Init_FreeType(&ft)) {
    FATAL("Error: Could not initialize FreeType library\n");
  }

  // Load text
  FT_Face face;
  if (FT_New_Face(ft, "./fonts/Inconsolata-Regular.ttf", 0, &face)) {
    FATAL("Error: Failed to load text\n");
  }

  // Set the text size (width and height in pixels)
  FT_Set_Pixel_Sizes(face, 0, text->size);

  // Disable byte-alignment restriction
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // Setup the standard 128 ASCII characters
  for (size_t c = 0; c < 128; ++c) {
    // Load character glyph
    if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
      FATAL("ERROR::FREETYTPE: Failed to load Glyph");
      continue;
    }

    // text details
    const gl_int_t ft_width = face->glyph->bitmap.width;
    const gl_int_t ft_height = face->glyph->bitmap.rows;
    const void *ft_data = face->glyph->bitmap.buffer;

    // Generate texture
    unsigned int texture_id;
    const gl_enum_t target = GL_TEXTURE_2D;
    const gl_enum_t ifmt = GL_RED;
    const gl_enum_t fmt = GL_RED;
    const gl_enum_t type = GL_UNSIGNED_BYTE;

    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(target, 0, ifmt, ft_width, ft_height, 0, fmt, type, ft_data);

    // Set texture options
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Store character for later use
    text->data[c].texture_id = texture_id;
    text->data[c].size[0] = face->glyph->bitmap.width;
    text->data[c].size[1] = face->glyph->bitmap.rows;
    text->data[c].bearing[0] = face->glyph->bitmap_left;
    text->data[c].bearing[1] = face->glyph->bitmap_top;
    text->data[c].offset = face->glyph->advance.x;
  }
  glBindTexture(GL_TEXTURE_2D, 0);
  FT_Done_Face(face);
  FT_Done_FreeType(ft);

  // VAO
  glGenVertexArrays(1, &text->entity.VAO);
  glBindVertexArray(text->entity.VAO);

  // VBO
  glGenBuffers(1, &text->entity.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, text->entity.VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gl_text_cleanup(gl_text_t *text) {
  // Clean up
  gl_entity_cleanup(&text->entity);
}

gl_text_t *gl_text_malloc(void) {
  gl_text_t *text = MALLOC(gl_text_t, 1);
  gl_text_setup(text);
  return text;
}

void gl_text_free(gl_text_t *text) {
  if (text == NULL) {
    return;
  }
  gl_text_cleanup(text);
  free(text);
}

void gl_text_draw(gl_text_t *text,
                  gl_camera_t *camera,
                  const char *s,
                  float x,
                  const float y,
                  const float scale) {
  // Setup projection matrix
  const gl_float_t left = 0.0f;
  const gl_float_t right = *(camera->window_width);
  const gl_float_t bottom = 0.0f;
  const gl_float_t top = *(camera->window_height);
  const gl_float_t znear = -1.0f;
  const gl_float_t zfar = 1.0f;
  gl_ortho(left, right, bottom, top, znear, zfar, text->entity.P);

  // Activate shader
  glUseProgram(text->entity.program_id);
  gl_prog_set_mat4(text->entity.program_id, "projection", text->entity.P);
  gl_prog_set_color(text->entity.program_id, "text_color", text->color);
  glActiveTexture(GL_TEXTURE0);
  glBindVertexArray(text->entity.VAO);

  // Render text
  for (size_t i = 0; i < strlen(s); ++i) {
    gl_char_t *ch = &text->data[(int) s[i]];
    const float xpos = x + ch->bearing[0] * scale;
    const float ypos = y - (ch->size[1] - ch->bearing[1]) * scale;
    const float w = ch->size[0] * scale;
    const float h = ch->size[1] * scale;

    // Update VBO for each character
    // clang-format off
    float vertices[6][4] = {
        {xpos,     ypos + h, 0.0f, 0.0f},
        {xpos,     ypos,     0.0f, 1.0f},
        {xpos + w, ypos,     1.0f, 1.0f},
        {xpos,     ypos + h, 0.0f, 0.0f},
        {xpos + w, ypos,     1.0f, 1.0f},
        {xpos + w, ypos + h, 1.0f, 0.0f},
    };
    // clang-format on

    // Render glyph texture over quad
    glBindTexture(GL_TEXTURE_2D, ch->texture_id);

    // Update content of VBO memory
    glBindBuffer(GL_ARRAY_BUFFER, text->entity.VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Render quad
    glDrawArrays(GL_TRIANGLES, 0, 6);
    // Offset cursors for next glyph (Note: advance is number of 1/64 pixels)

    // Bitshift by 6 to get value in pixels (2^6 = 64)
    x += (ch->offset >> 6) * scale;
  }

  // Clean up
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);
}

// GL IMAGE //////////////////////////////////////////////////////////////////

#define GL_IMAGE_VS                                                            \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec2 in_pos;\n"                                    \
  "layout (location = 1) in vec2 in_tex_coord;\n"                              \
  "out vec2 tex_coord;\n"                                                      \
  "void main() {\n"                                                            \
  "  gl_Position = vec4(in_pos, 0.0, 1.0);\n"                                  \
  "  tex_coord = in_tex_coord;\n"                                              \
  "}\n"

#define GL_IMAGE_FS                                                            \
  "#version 330 core\n"                                                        \
  "in vec2 tex_coord;\n"                                                       \
  "out vec4 frag_color;\n"                                                     \
  "uniform sampler2D texture1;\n"                                              \
  "void main() {\n"                                                            \
  "  frag_color = texture(texture1, tex_coord);\n"                             \
  "}\n"

gl_image_t *gl_image_malloc(void) {
  // MALLOC
  gl_image_t *image = MALLOC(gl_image_t, 1);

  // Shader program
  image->program_id = gl_prog_setup(GL_IMAGE_VS, GL_IMAGE_FS, NULL);
  if (image->program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Rectangle vertices and texture coordinates
  // clang-format off
  const gl_float_t vertices[4 * 4] = {
     // positions // texture coords
     0.5f,  0.5f, 1.0f,  1.0f, // top right
     0.5f, -0.5f, 1.0f,  0.0f, // bottom right
    -0.5f, -0.5f, 0.0f,  0.0f, // bottom left
    -0.5f,  0.5f, 0.0f,  1.0f  // top left
  };
  const gl_uint_t indices[2 * 3] = {
    0, 3, 1, // First Triangle
    2, 1, 3  // Second Triangle
  };
  const size_t num_vertices = 4;
  const size_t vertex_size = sizeof(gl_float_t) * 4;
  const size_t vbo_size = sizeof(vertices);
  const size_t ebo_size = sizeof(indices);
  assert(vbo_size == vertex_size * num_vertices);
  assert(ebo_size == sizeof(gl_uint_t) * 6);
  // clang-format on

  // VAO
  glGenVertexArrays(1, &image->vao);
  glBindVertexArray(image->vao);
  assert(image->vao != 0);

  // VBO
  glGenBuffers(1, &image->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, image->vbo);
  glBufferData(GL_ARRAY_BUFFER, vbo_size, vertices, GL_STATIC_DRAW);
  assert(image->vbo != 0);

  // EBO
  glGenBuffers(1, &image->ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, image->ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, ebo_size, indices, GL_STATIC_DRAW);
  assert(image->ebo != 0);

  // Position attribute
  const void *pos_offset = (void *) (sizeof(gl_float_t) * 0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);

  // Texture coordinate attribute
  const void *tex_offset = (void *) (sizeof(gl_float_t) * 2);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, vertex_size, tex_offset);
  glEnableVertexAttribArray(1);

  // Load texture
  int width = 0;
  int height = 0;
  int channels = 0;
  const char *image_path = "/tmp/container.jpg";
  stbi_set_flip_vertically_on_load(1);
  unsigned char *data = stbi_load(image_path, &width, &height, &channels, 0);
  if (data) {
    glGenTextures(1, &image->texture_id);
    glBindTexture(GL_TEXTURE_2D, image->texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 width,
                 height,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 data);
    glGenerateMipmap(GL_TEXTURE_2D);
  } else {
    FATAL("Failed to load image [%s]!\n", image_path);
  }
  stbi_image_free(data);

  // Unbind VBO and VAO
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  return image;
}

void gl_image_free(gl_image_t *image) {
  glDeleteVertexArrays(1, &image->vao);
  glDeleteBuffers(1, &image->vbo);
  glDeleteBuffers(1, &image->ebo);
  glDeleteProgram(image->program_id);
  free(image);
}

void gl_image_draw(gl_image_t *image) {
  glUseProgram(image->program_id);
  glBindVertexArray(image->vao);
  glBindTexture(GL_TEXTURE_2D, image->texture_id);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

/******************************************************************************
 * GL-MESH
 *****************************************************************************/

void gl_mesh_setup(gl_mesh_t *mesh,
                   gl_vertex_t *vertices,
                   const int num_vertices,
                   unsigned int *indices,
                   const int num_indices,
                   gl_texture_t *textures,
                   const int num_textures) {
  // Setup
  mesh->vertices = vertices;
  mesh->indices = indices;
  mesh->textures = textures;
  mesh->num_vertices = num_vertices;
  mesh->num_indices = num_indices;
  mesh->num_textures = num_textures;

  // VAO
  glGenVertexArrays(1, &mesh->VAO);
  glBindVertexArray(mesh->VAO);

  // VBO
  glGenBuffers(1, &mesh->VBO);
  glBindBuffer(GL_ARRAY_BUFFER, mesh->VBO);
  glBufferData(GL_ARRAY_BUFFER,
               sizeof(gl_vertex_t) * num_vertices,
               &vertices[0],
               GL_STATIC_DRAW);

  // EBO
  glGenBuffers(1, &mesh->EBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               sizeof(unsigned int) * num_indices,
               &indices[0],
               GL_STATIC_DRAW);

  // Vertex positions
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        sizeof(gl_vertex_t),
                        (void *) 0);

  // Vertex normals
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        sizeof(gl_vertex_t),
                        (void *) offsetof(gl_vertex_t, normal));

  // Vertex texture coords
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2,
                        2,
                        GL_FLOAT,
                        GL_FALSE,
                        sizeof(gl_vertex_t),
                        (void *) offsetof(gl_vertex_t, tex_coords));

  // Clean up
  glBindVertexArray(0);
}

void gl_mesh_draw(const gl_mesh_t *mesh, const gl_uint_t shader) {
  // bind appropriate textures
  unsigned int num_diffuse = 1;
  unsigned int num_specular = 1;
  unsigned int num_normal = 1;
  unsigned int num_height = 1;

  for (int i = 0; i < mesh->num_textures; i++) {
    // Active proper texture unit before binding
    glActiveTexture(GL_TEXTURE0 + i);

    // Form texture unit (the N in diffuse_textureN)
    char texture_unit[120] = {0};
    if (strcmp(mesh->textures[i].type, "texture_diffuse") == 0) {
      sprintf(texture_unit, "%s%d", mesh->textures[i].type, num_diffuse++);
    } else if (strcmp(mesh->textures[i].type, "texture_specular") == 0) {
      sprintf(texture_unit, "%s%d", mesh->textures[i].type, num_specular++);
    } else if (strcmp(mesh->textures[i].type, "texture_normal") == 0) {
      sprintf(texture_unit, "%s%d", mesh->textures[i].type, num_normal++);
    } else if (strcmp(mesh->textures[i].type, "texture_height") == 0) {
      sprintf(texture_unit, "%s%d", mesh->textures[i].type, num_height++);
    }

    // Set the sampler to the correct texture unit and bind the texture
    glUniform1i(glGetUniformLocation(shader, texture_unit), i);
    glBindTexture(GL_TEXTURE_2D, mesh->textures[i].id);
  }

  // Draw mesh
  glBindVertexArray(mesh->VAO);
  glDrawElements(GL_TRIANGLES, mesh->num_indices, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);

  // Set everything back to defaults once configured
  glActiveTexture(GL_TEXTURE0);
}

/******************************************************************************
 * GL-MODEL
 *****************************************************************************/

#define GL_MODEL_VS                                                            \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec3 in_pos;\n"                                    \
  "layout (location = 1) in vec3 in_normal;\n"                                 \
  "layout (location = 2) in vec2 in_tex_coords;\n"                             \
  "out vec2 tex_coords;\n"                                                     \
  "out vec3 frag_pos;\n"                                                       \
  "out vec3 normal;\n"                                                         \
  "uniform mat4 model;\n"                                                      \
  "uniform mat4 view;\n"                                                       \
  "uniform mat4 projection;\n"                                                 \
  "void main() {\n"                                                            \
  "  tex_coords = in_tex_coords;\n"                                            \
  "  frag_pos = vec3(model * vec4(in_pos, 1.0));\n"                            \
  "  normal = mat3(transpose(inverse(model))) * in_normal;\n"                  \
  "  gl_Position = projection * view * model * vec4(in_pos, 1.0);\n"           \
  "}\n"

#define GL_MODEL_FS                                                            \
  "#version 330 core\n"                                                        \
  "in vec2 tex_coords;\n"                                                      \
  "out vec4 frag_color;\n"                                                     \
  "uniform sampler2D texture_diffuse1;\n"                                      \
  "void main() {\n"                                                            \
  "  frag_color = texture(texture_diffuse1, tex_coords);\n"                    \
  "}\n"

static unsigned int gl_texture_load(const char *model_dir,
                                    const char *texture_fname) {
  // File fullpath
  char filepath[9046] = {0};
  strcat(filepath, model_dir);
  strcat(filepath, "/");
  strcat(filepath, texture_fname);

  // Generate texture ID
  unsigned int texture_id;
  glGenTextures(1, &texture_id);

  // Load image
  stbi_set_flip_vertically_on_load(1);
  int width = 0;
  int height = 0;
  int channels = 0;
  unsigned char *data = stbi_load(filepath, &width, &height, &channels, 0);
  if (data) {
    // Image format
    gl_enum_t format;
    if (channels == 1) {
      format = GL_RED;
    } else if (channels == 3) {
      format = GL_RGB;
    } else if (channels == 4) {
      format = GL_RGBA;
    } else {
      printf("Invalid number of channels: %d\n", channels);
      return -1;
    }

    // Load image to texture ID
    // clang-format off
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // clang-format on

  } else {
    printf("Texture failed to load: [%s]\n", filepath);
    return -1;
  }

  // Clean up
  stbi_image_free(data);

  return texture_id;
}

static void assimp_load_textures(const struct aiMaterial *material,
                                 const enum aiTextureType type,
                                 const char *model_dir,
                                 gl_texture_t *textures,
                                 int *textures_length) {
  // Setup
  int texture_index = MAX(*textures_length - 1, 0);
  const int num_textures = aiGetMaterialTextureCount(material, type);

  // Type name
  char type_name[30] = {0};
  switch (type) {
    case aiTextureType_DIFFUSE:
      strcpy(type_name, "texture_diffuse");
      break;
    case aiTextureType_SPECULAR:
      strcpy(type_name, "texture_specular");
      break;
    case aiTextureType_HEIGHT:
      strcpy(type_name, "texture_height");
      break;
    case aiTextureType_AMBIENT:
      strcpy(type_name, "texture_ambient");
      break;
    default:
      FATAL("Not Implemented!");
      break;
  }

  // Load texture
  for (int index = 0; index < num_textures; index++) {
    struct aiString texture_fname;
    enum aiTextureMapping *mapping = NULL;
    unsigned int *uvindex = NULL;
    ai_real *blend = NULL;
    enum aiTextureOp *op = NULL;
    enum aiTextureMapMode *mapmode = NULL;
    unsigned int *flags = NULL;
    aiGetMaterialTexture(material,
                         type,
                         index,
                         &texture_fname,
                         mapping,
                         uvindex,
                         blend,
                         op,
                         mapmode,
                         flags);

    // Check if texture was loaded before and if so, continue to next iteration
    // int load_texture = 1;
    // for (unsigned int j = 0; j < textures_loaded.size(); j++) {
    //   if (strcmp(textures_loaded[j].path.data(), str.C_Str()) == 0) {
    //     // textures.push_back(textures_loaded[j]);
    //     load_texture = 0;
    //     break;
    //   }
    // }

    // Load texture
    // if (load_texture) {
    //   Texture texture;
    //   texture.id = TextureFromFile(str.C_Str(), this->directory);
    //   texture.type = type_name;
    //   texture.path = str.C_Str();
    //   textures.push_back(texture);
    //   textures_loaded.push_back(texture);
    // }

    textures[texture_index].id = gl_texture_load(model_dir, texture_fname.data);
    strcpy(textures[texture_index].type, type_name);
    strcpy(textures[texture_index].path, texture_fname.data);
    texture_index++;
    (*textures_length)++;
  }
}

static void assimp_load_mesh(const struct aiMesh *mesh,
                             const struct aiMaterial *material,
                             gl_model_t *model) {
  // For each mesh vertices
  const int num_vertices = mesh->mNumVertices;
  gl_vertex_t *vertices = MALLOC(gl_vertex_t, num_vertices);
  for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
    // Position
    vertices[i].position[0] = mesh->mVertices[i].x;
    vertices[i].position[1] = mesh->mVertices[i].y;
    vertices[i].position[2] = mesh->mVertices[i].z;

    // Normal
    if (mesh->mNormals != NULL && mesh->mNumVertices > 0) {
      vertices[i].normal[0] = mesh->mNormals[i].x;
      vertices[i].normal[1] = mesh->mNormals[i].y;
      vertices[i].normal[2] = mesh->mNormals[i].z;
    }

    // Texture coordinates
    if (mesh->mTextureCoords[0]) {
      // Texture coordinates
      vertices[i].tex_coords[0] = mesh->mTextureCoords[0][i].x;
      vertices[i].tex_coords[1] = mesh->mTextureCoords[0][i].y;
      // Note: A vertex can contain up to 8 different texture coordinates. We
      // thus make the assumption that we won't use models where a vertex can
      // have multiple texture coordinates so we always take the first set (0).
    } else {
      // Default Texture coordinates
      vertices[i].tex_coords[0] = 0.0f;
      vertices[i].tex_coords[1] = 0.0f;
    }

    // Tangent
    if (mesh->mTangents) {
      vertices[i].tangent[0] = mesh->mTangents[i].x;
      vertices[i].tangent[1] = mesh->mTangents[i].y;
      vertices[i].tangent[2] = mesh->mTangents[i].z;
    }

    // Bitangent
    if (mesh->mBitangents) {
      vertices[i].bitangent[0] = mesh->mBitangents[i].x;
      vertices[i].bitangent[1] = mesh->mBitangents[i].y;
      vertices[i].bitangent[2] = mesh->mBitangents[i].z;
    }
  }

  // For each mesh face
  // -- Determine number of indices
  size_t num_indices = 0;
  for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
    num_indices += mesh->mFaces[i].mNumIndices;
  }
  // -- Form indices array
  unsigned int *indices = MALLOC(unsigned int, num_indices);
  int index_counter = 0;
  for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
    for (unsigned int j = 0; j < mesh->mFaces[i].mNumIndices; j++) {
      indices[index_counter] = mesh->mFaces[i].mIndices[j];
      index_counter++;
    }
  }

  // Process texture materials
  // struct aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
  // Note: we assume a convention for sampler names in the shaders. Each
  // diffuse texture should be named as 'texture_diffuseN' where N is a
  // sequential number ranging from 1 to MAX_SAMPLER_NUMBER. Same applies to
  // other texture as the following list summarizes:
  // diffuse: texture_diffuseN
  // specular: texture_specularN
  // normal: texture_normalN

  // -- Get total number of textures
  int num_textures = 0;
  num_textures += aiGetMaterialTextureCount(material, aiTextureType_DIFFUSE);
  num_textures += aiGetMaterialTextureCount(material, aiTextureType_SPECULAR);
  num_textures += aiGetMaterialTextureCount(material, aiTextureType_HEIGHT);
  num_textures += aiGetMaterialTextureCount(material, aiTextureType_AMBIENT);

  // -- Load textures
  const char *model_dir = model->model_dir;
  int textures_length = 0;
  gl_texture_t *textures = MALLOC(gl_texture_t, num_textures);

  assimp_load_textures(material,
                       aiTextureType_DIFFUSE,
                       model_dir,
                       textures,
                       &textures_length);
  assimp_load_textures(material,
                       aiTextureType_SPECULAR,
                       model_dir,
                       textures,
                       &textures_length);
  assimp_load_textures(material,
                       aiTextureType_HEIGHT,
                       model_dir,
                       textures,
                       &textures_length);
  assimp_load_textures(material,
                       aiTextureType_AMBIENT,
                       model_dir,
                       textures,
                       &textures_length);

  // Form Mesh
  const int mesh_index = model->num_meshes;
  gl_mesh_setup(&model->meshes[mesh_index],
                vertices,
                num_vertices,
                indices,
                num_indices,
                textures,
                num_textures);
  model->num_meshes++;
}

static void assimp_load_model(const struct aiScene *scene,
                              const struct aiNode *node,
                              gl_model_t *model) {
  // Process each mesh located at the current node
  for (unsigned int i = 0; i < node->mNumMeshes; i++) {
    // The node object only contains indices to index the actual objects in the
    // scene. The scene contains all the data, node is just to keep stuff
    // organized (like relations between nodes).
    struct aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
    struct aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
    assimp_load_mesh(mesh, material, model);
  }

  // After processing all of the meshes (if any) we then recursively process
  // each of the children nodes
  for (unsigned int i = 0; i < node->mNumChildren; i++) {
    assimp_load_model(scene, node->mChildren[i], model);
  }
}

static int assimp_num_meshes(const struct aiNode *node) {
  int num_meshes = node->mNumMeshes;
  for (unsigned int i = 0; i < node->mNumChildren; i++) {
    num_meshes += assimp_num_meshes(node->mChildren[i]);
  }
  return num_meshes;
}

gl_model_t *gl_model_load(const char *model_path) {
  // Check model file
  if (file_exists(model_path) != 0) {
    return NULL;
  }

  // Malloc
  gl_model_t *model = MALLOC(gl_model_t, 1);

  // Entity transform
  gl_eye(model->T, 4, 4);
  model->T[12] = 0.0;
  model->T[13] = 0.0;
  model->T[14] = 0.0;

  // Shader program
  model->program_id = gl_prog_setup(GL_MODEL_VS, GL_MODEL_FS, NULL);
  if (model->program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Using assimp to load model
  const struct aiScene *scene =
      aiImportFile(model_path, aiProcessPreset_TargetRealtime_MaxQuality);
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE ||
      !scene->mRootNode) {
    printf("Failed to load model: %s\n", model_path);
    return NULL;
  }

  // Get model directory
  char path[9046] = {0};
  strcpy(path, model_path);
  char *model_dir = dirname(path);
  if (model_dir == NULL) {
    printf("Failed to get directory name of [%s]!", model_path);
    return NULL;
  }
  strcpy(model->model_dir, model_dir);

  // Load model
  const int num_meshes = assimp_num_meshes(scene->mRootNode);
  model->meshes = MALLOC(gl_mesh_t, num_meshes);
  model->num_meshes = 0;
  assimp_load_model(scene, scene->mRootNode, model);

  // Clean up
  aiReleaseImport(scene);

  return model;
}

void gl_model_free(gl_model_t *model) {
  if (model == NULL) {
    return;
  }

  for (int i = 0; i < model->num_meshes; i++) {
    free(model->meshes[i].vertices);
    free(model->meshes[i].indices);
    free(model->meshes[i].textures);
  }
  free(model->meshes);

  free(model);
  model = NULL;
}

void gl_model_draw(const gl_model_t *model, const gl_camera_t *camera) {
  glUseProgram(model->program_id);
  gl_prog_set_mat4(model->program_id, "projection", camera->P);
  gl_prog_set_mat4(model->program_id, "view", camera->V);
  gl_prog_set_mat4(model->program_id, "model", model->T);

  float light_pos[3] = {0, 10, 0};
  float light_color[3] = {1, 1, 1};
  float object_color[3] = {1, 1, 1};
  gl_prog_set_vec3(model->program_id, "lightPos", light_pos);
  gl_prog_set_vec3(model->program_id, "viewPos", camera->position);
  gl_prog_set_vec3(model->program_id, "lightColor", light_color);
  gl_prog_set_vec3(model->program_id, "objectColor", object_color);

  for (int i = 0; i < model->num_meshes; i++) {
    gl_mesh_draw(&model->meshes[i], model->program_id);
  }
}

/******************************************************************************
 * GUI
 *****************************************************************************/

// WINDOW ////////////////////////////////////////////////////////////////////

void window_callback(GLFWwindow *window, int width, int height) {
  gui_t *gui = (gui_t *) glfwGetWindowUserPointer(window);
  gui->window_width = width;
  gui->window_height = height;
  glViewport(0, 0, width, height);
}

void gui_process_input(GLFWwindow *window) {
  const float camera_speed = 0.1f;
  gui_t *gui = (gui_t *) glfwGetWindowUserPointer(window);

  // Handle keyboard events
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS ||
      glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
    gui->loop = 0;
  }

  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    if (gui->camera.view_mode == FPS) {
      gui->camera.position[0] += camera_speed * gui->camera.front[0];
      gui->camera.position[1] += camera_speed * gui->camera.front[1];
      gui->camera.position[2] += camera_speed * gui->camera.front[2];
    } else if (gui->camera.view_mode == ORBIT) {
      gui->camera.pitch += 0.01;
      gui->camera.pitch =
          (gui->camera.pitch >= M_PI) ? M_PI : gui->camera.pitch;
      gui->camera.pitch =
          (gui->camera.pitch <= 0.0f) ? 0.0f : gui->camera.pitch;
    }
  }

  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    if (gui->camera.view_mode == FPS) {
      gui->camera.position[0] -= camera_speed * gui->camera.front[0];
      gui->camera.position[1] -= camera_speed * gui->camera.front[1];
      gui->camera.position[2] -= camera_speed * gui->camera.front[2];
    } else if (gui->camera.view_mode == ORBIT) {
      gui->camera.pitch -= 0.01;
      gui->camera.pitch =
          (gui->camera.pitch >= M_PI) ? M_PI : gui->camera.pitch;
      gui->camera.pitch =
          (gui->camera.pitch <= 0.0f) ? 0.0f : gui->camera.pitch;
    }
  }

  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    if (gui->camera.view_mode == FPS) {
      gl_float_t camera_left[3] = {0};
      gl_vec3_cross(gui->camera.front, gui->camera.up, camera_left);
      gl_normalize(camera_left, 3);
      gui->camera.position[0] -= camera_left[0] * camera_speed;
      gui->camera.position[1] -= camera_left[1] * camera_speed;
      gui->camera.position[2] -= camera_left[2] * camera_speed;
    } else if (gui->camera.view_mode == ORBIT) {
      gui->camera.yaw -= 0.01;
      gui->camera.yaw = (gui->camera.yaw >= M_PI) ? M_PI : gui->camera.yaw;
      gui->camera.yaw = (gui->camera.yaw <= -M_PI) ? -M_PI : gui->camera.yaw;
    }
  }

  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    if (gui->camera.view_mode == FPS) {
      gl_float_t camera_left[3] = {0};
      gl_vec3_cross(gui->camera.front, gui->camera.up, camera_left);
      gl_normalize(camera_left, 3);
      gui->camera.position[0] += camera_left[0] * camera_speed;
      gui->camera.position[1] += camera_left[1] * camera_speed;
      gui->camera.position[2] += camera_left[2] * camera_speed;
    } else if (gui->camera.view_mode == ORBIT) {
      gui->camera.yaw += 0.01;
      gui->camera.yaw = (gui->camera.yaw >= M_PI) ? M_PI : gui->camera.yaw;
      gui->camera.yaw = (gui->camera.yaw <= -M_PI) ? -M_PI : gui->camera.yaw;
    }
  }

  if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS) {
    if (gui->camera.view_mode == FPS) {
      gl_camera_zoom(&gui->camera, 1.0, 0, 0.1);
    } else if (gui->camera.view_mode == ORBIT) {
      gui->camera.radius += 0.1;
      gui->camera.radius =
          (gui->camera.radius <= 0.01) ? 0.01 : gui->camera.radius;
    }
  }

  if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS) {
    if (gui->camera.view_mode == FPS) {
      gl_camera_zoom(&gui->camera, 1.0, 0, -0.1);
    } else if (gui->camera.view_mode == ORBIT) {
      gui->camera.radius -= 0.1;
      gui->camera.radius =
          (gui->camera.radius <= 0.01) ? 0.01 : gui->camera.radius;
    }
  }

  // Handle mouse cursor events
  glfwGetCursorPos(window, &gui->cursor_x, &gui->cursor_y);
  const float dx = gui->cursor_x - gui->last_cursor_x;
  const float dy = gui->cursor_y - gui->last_cursor_y;
  gui->last_cursor_x = gui->cursor_x;
  gui->last_cursor_y = gui->cursor_y;

  const int button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
  const int button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
  gui->left_click = (button_left == GLFW_PRESS);
  gui->right_click = (button_right == GLFW_PRESS);

  if (gui->left_click) {
    // Rotate camera
    if (gui->last_cursor_set == 0) {
      gui->last_cursor_set = 1;
    } else if (gui->last_cursor_set) {
      gl_camera_rotate(&gui->camera, gui->mouse_sensitivity, dx, dy);
    }
  } else if (gui->right_click) {
    // Pan camera
    if (gui->last_cursor_set == 0) {
      gui->last_cursor_set = 1;
    } else if (gui->last_cursor_set) {
      gl_camera_pan(&gui->camera, gui->mouse_sensitivity, dx, dy);
    }
  } else {
    // Reset cursor
    gui->left_click = 0;
    gui->right_click = 0;
    gui->last_cursor_set = 0;
    gui->last_cursor_x = 0.0;
    gui->last_cursor_y = 0.0;
  }
}

void gui_setup(gui_t *gui) {
  // GLFW
  if (!glfwInit()) {
    printf("Failed to initialize glfw!\n");
    exit(EXIT_FAILURE);
  }

  // Window settings
  glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, 1);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  gui->window = glfwCreateWindow(gui->window_width,
                                 gui->window_height,
                                 gui->window_title,
                                 NULL,
                                 NULL);
  if (!gui->window) {
    printf("Failed to create glfw window!\n");
    glfwTerminate();
    exit(EXIT_FAILURE);
  }
  glfwMakeContextCurrent(gui->window);
  glfwSetWindowUserPointer(gui->window, gui);
  glfwSetWindowSizeCallback(gui->window, window_callback);

  // GLAD
  if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
    printf("Failed to load GL functions!\n");
    return;
  }

  // OpenGL functions
  glEnable(GL_DEBUG_OUTPUT);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  assert(glIsEnabled(GL_PROGRAM_POINT_SIZE));
  assert(glIsEnabled(GL_LINE_SMOOTH));
  assert(glIsEnabled(GL_DEPTH_TEST));
  assert(glIsEnabled(GL_CULL_FACE));
  assert(glIsEnabled(GL_BLEND));

  // Camera
  gl_camera_setup(&gui->camera, &gui->window_width, &gui->window_height);
  gui->camera.position[0] = 0.0f;
  gui->camera.position[1] = 4.0f;
  gui->camera.position[2] = -5.0f;
  gui->movement_speed = 50.0f;
  gui->mouse_sensitivity = 0.02f;

  // Cursor
  gui->left_click = 0;
  gui->right_click = 0;
  gui->last_cursor_set = 0;
  gui->last_cursor_x = 0.0f;
  gui->last_cursor_y = 0.0f;
}

void gui_reset(gui_t *gui) {
  // Camera
  gui->movement_speed = 50.0f;
  gui->mouse_sensitivity = 0.02f;

  // Cursor
  gui->left_click = 0;
  gui->right_click = 0;
  gui->last_cursor_set = 0;
  gui->last_cursor_x = 0.0f;
  gui->last_cursor_y = 0.0f;
}

void gui_loop(gui_t *gui) {
  // Cube
  const gl_float_t cube_size = 0.5f;
  const gl_float_t cube_pos[3] = {0.0, 0.0, 0.0};
  const gl_float_t cube_color[3] = {0.9, 0.4, 0.2};
  const gl_float_t outline_color[3] = {1.0, 1.0, 1.0};
  gl_cube_t *cube =
      gl_cube_malloc(cube_size, cube_pos, cube_color, outline_color);

  // Rect
  gl_rect_t *rect = gl_rect_malloc((gl_bounds_t){0, 0, 10, 10});

  // Camera frame
  const gl_float_t cf_pos[3] = {0.0, 0.0, 0.0};
  const gl_float_t cf_size = 0.5f;
  const gl_float_t cf_color[3] = {0.9, 0.4, 0.2};
  gl_cframe_t *cf = gl_cframe_malloc(cf_pos, cf_size, cf_color);

  // Axes
  gl_axes3d_t *axes = gl_axes3d_malloc();

  // Grid
  gl_grid3d_t *grid = gl_grid3d_malloc();

  // Points
  const gl_float_t points_color[3] = {1.0, 0.0, 0.0};
  const gl_float_t points_size = 1.0;
  const size_t num_points = 100000;
  gl_float_t *points_data = MALLOC(gl_float_t, num_points * 3);
  for (size_t i = 0; i < num_points; ++i) {
    points_data[i * 3 + 0] = gl_randf(-1.0f, 1.0f);
    points_data[i * 3 + 1] = gl_randf(-1.0f, 1.0f);
    points_data[i * 3 + 2] = gl_randf(-1.0f, 1.0f);
  }
  gl_points3d_t *points =
      gl_points3d_malloc(points_data, num_points, points_size, points_color);
  free(points_data);

  // Line
  const gl_float_t line_width = 5.0f;
  const gl_float_t line_color[3] = {1.0f, 0.0f, 0.0f};
  const size_t line_num_points = 1000;
  const float radius = 3.0f;
  const float dtheta = 2 * M_PI / line_num_points;
  float theta = 0.0f;
  float *line_points = MALLOC(float, line_num_points * 3);
  for (size_t i = 0; i < line_num_points; ++i) {
    line_points[i * 3 + 0] = radius * sin(theta);
    line_points[i * 3 + 1] = 0.0f;
    line_points[i * 3 + 2] = radius * cos(theta);
    theta += 0.01;
  }
  gl_line3d_t *line =
      gl_line3d_malloc(line_points, line_num_points, line_width, line_color);
  free(line_points);

  // Text
  gl_text_t *text = gl_text_malloc();

  // Image
  gl_image_t *image = gl_image_malloc();

  // Button
  ui_button_t button;
  ui_button_setup(&button);

  // Render loop
  gui->loop = 1;
  glfwMakeContextCurrent(gui->window);

  while (gui->loop) {
    // Clear rendering states
    glClear(GL_DEPTH_BUFFER_BIT);
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Draw
    // gl_rect_draw(rect, &gui->camera);
    // gl_cube_draw(cube, &gui->camera);
    // gl_cframe_draw(cf, &gui->camera);
    // gl_axes3d_draw(axes, &gui->camera);
    // gl_grid3d_draw(grid, &gui->camera);
    // gl_line3d_draw(line, &gui->camera);
    // gl_points3d_draw(points, &gui->camera, num_points);
    // gl_text_draw(text, &gui->camera, "Here2!", 500.0f, 500.0f, 1.0f);
    // gl_image_draw(image);
    ui_button_draw(gui, &button);
    gl_text_draw(text, &gui->camera, "Here!", 150.0f, 100.0f, 1.0f);

    // Update
    glfwPollEvents();
    gui_process_input(gui->window);
    gl_camera_update(&gui->camera);
    glfwSwapBuffers(gui->window);
  }

  // Clean up
  gl_rect_free(rect);
  gl_cube_free(cube);
  gl_cframe_free(cf);
  gl_axes3d_free(axes);
  gl_grid3d_free(grid);
  gl_points3d_free(points);
  gl_line3d_free(line);
  gl_text_free(text);
  gl_image_free(image);
  glfwTerminate();
}

// UI-BUTTON /////////////////////////////////////////////////////////////////

#define UI_BUTTON_VS                                                           \
  "#version 330 core\n"                                                        \
  "layout (location = 0) in vec2 in_pos;\n"                                    \
  "uniform float window_width;\n"                                              \
  "uniform float window_height;\n"                                             \
  "uniform float button_width;\n"                                              \
  "uniform float button_height;\n"                                             \
  "void main() {\n"                                                            \
  "  gl_Position = vec4(in_pos, 0.0, 1.0);\n"                                  \
  "}\n"

#define UI_BUTTON_FS                                                           \
  "#version 330 core\n"                                                        \
  "uniform vec3 color;\n"                                                      \
  "out vec4 frag_color;\n"                                                     \
  "void main() {\n"                                                            \
  "  frag_color = vec4(color, 1.0f);\n"                                        \
  "}\n"

void pixel2ndc(const int x_px,
               const int y_px,
               const float window_width,
               const float window_height,
               gl_float_t *x_ndc,
               gl_float_t *y_ndc) {
  *x_ndc = 2.0f * x_px / window_width - 1.0f;
  *y_ndc = 1.0f - 2.0f * y_px / window_height;
}

void ui_button_setup(ui_button_t *button) {
  button->color = (gl_color_t){1.0f, 1.0f, 1.0f};
  button->color_hover = (gl_color_t){1.0f, 0.0f, 0.0f};
  button->color_press = (gl_color_t){0.0f, 0.0f, 1.0f};
  button->bounds.x = 100;
  button->bounds.y = 600;
  button->bounds.w = 150.0f;
  button->bounds.h = 100.0f;

  button->program_id = -1;
  button->vao = -1;
  button->vbo = -1;
  button->ebo = -1;

  // Shader program
  button->program_id = gl_prog_setup(UI_BUTTON_VS, UI_BUTTON_FS, NULL);
  if (button->program_id == GL_FALSE) {
    FATAL("Failed to create shaders!");
  }

  // Vertices
  // clang-format off
  const int x = button->bounds.x;
  const int y = button->bounds.y;
  const int w = button->bounds.w;
  const int h = button->bounds.h;
  float vertices[4 * 2] = {0};
  pixel2ndc(x    , y + h, 1024, 768, &vertices[0], &vertices[1]); // Bottom left
  pixel2ndc(x + w, y + h, 1024, 768, &vertices[2], &vertices[3]); // Bottom right
  pixel2ndc(x + w, y    , 1024, 768, &vertices[4], &vertices[5]); // Top right
  pixel2ndc(x    , y    , 1024, 768, &vertices[6], &vertices[7]); // Top left
  // clang-format on

  // VAO
  glGenVertexArrays(1, &button->vao);
  glBindVertexArray(button->vao);

  // VBO
  glGenBuffers(1, &button->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, button->vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  // Position attribute
  size_t pos_size = sizeof(float) * 2;
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, pos_size, pos_offset);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void ui_button_cleanup(const ui_button_t *button) {
  glDeleteVertexArrays(1, &button->vao);
  glDeleteBuffers(1, &button->vbo);
}

static int ui_intercept(const gui_t *gui, const gl_bounds_t *bounds) {
  const int x = bounds->x;
  const int y = bounds->y;
  const int w = bounds->w;
  const int h = bounds->h;
  int within_x = (x <= gui->cursor_x && gui->cursor_x <= x + w);
  int within_y = (y <= gui->cursor_y && gui->cursor_y <= y + h);
  return (within_x && within_y) ? 1 : 0;
}

int ui_button_draw(const gui_t *gui, const ui_button_t *button) {
  // Button color
  const int button_hover = ui_intercept(gui, &button->bounds);
  const int button_pressed = gui->left_click;
  int button_on = 0;
  gl_color_t color = button->color;
  if (button_hover == 1 && button_pressed == 0) {
    color = button->color_hover;
  } else if (button_hover == 1 && button_pressed == 1) {
    color = button->color_press;
    button_on = 1;
  }

  // Shader program
  glUseProgram(button->program_id);
  gl_prog_set_color(button->program_id, "color", color);
  gl_prog_set_float(button->program_id, "window_width", gui->window_width);
  gl_prog_set_float(button->program_id, "window_height", gui->window_height);
  gl_prog_set_float(button->program_id, "button_width", button->bounds.w);
  gl_prog_set_float(button->program_id, "button_height", button->bounds.h);
  glBindVertexArray(button->vao);
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  glBindVertexArray(0);

  return button_on;
}

#endif // GUI_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef GUI_UNITTEST

#include <stdio.h>

// UNITESTS GLOBAL VARIABLES
static int num_tests = 0;
static int num_passed = 0;
static int num_failed = 0;

#define ENABLE_TERM_COLORS 0
#if ENABLE_TERM_COLORS == 1
#define TERM_RED "\x1B[1;31m"
#define TERM_GRN "\x1B[1;32m"
#define TERM_WHT "\x1B[1;37m"
#define TERM_NRM "\x1B[1;0m"
#else
#define TERM_RED
#define TERM_GRN
#define TERM_WHT
#define TERM_NRM
#endif

/**
 * Run unittests
 * @param[in] test_name Test name
 * @param[in] test_ptr Pointer to unittest
 */
void run_test(const char *test_name, int (*test_ptr)(void)) {
  if ((*test_ptr)() == 0) {
    printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
    fflush(stdout);
    num_passed++;
  } else {
    printf(TERM_RED "FAILED!\n" TERM_NRM);
    fflush(stdout);
    num_failed++;
  }
  num_tests++;
}

/**
 * Add unittest
 * @param[in] TEST Test function
 */
#define TEST(TEST_FN) run_test(#TEST_FN, TEST_FN);

/**
 * Unit-test assert
 * @param[in] TEST Test condition
 */
#define TEST_ASSERT(TEST)                                                      \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      printf(TERM_RED "ERROR!" TERM_NRM " [%s:%d] %s FAILED!\n",               \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST);                                                           \
      return -1;                                                               \
    }                                                                          \
  } while (0)

static GLFWwindow *test_setup(void) {
  // GLFW
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  const int win_w = 800;
  const int win_h = 800;
  const char *win_title = "Test";
  GLFWwindow *win = glfwCreateWindow(win_w, win_h, win_title, NULL, NULL);
  if (win == NULL) {
    FATAL("Failed to create GLFW window!\n!");
    glfwTerminate();
    return NULL;
  }
  glfwMakeContextCurrent(win);

  // GLAD
  if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
    FATAL("Failed to load GL context!\n!");
    return NULL;
  }

  // OpenGL Features
  glEnable(GL_CULL_FACE);

  return win;
}

static void test_teardown(GLFWwindow *window) {
  glfwTerminate();
  // free(window);
}

// TEST GLFW /////////////////////////////////////////////////////////////////

int test_glfw(void) {
  if (!glfwInit()) {
    printf("Cannot initialize GLFW\n");
    exit(EXIT_FAILURE);
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  GLFWwindow *window = glfwCreateWindow(640, 480, "Simple example", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }

  while (!glfwWindowShouldClose(window)) {
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  glfwTerminate();

  return 0;
}

// TEST OPENGL UTILS /////////////////////////////////////////////////////////

int test_gl_zeros(void) {
  // clang-format off
  gl_float_t A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  gl_float_t expected[3*3] = {0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0};
  // clang-format on
  gl_zeros(A, 3, 3);
  TEST_ASSERT(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_ones(void) {
  // clang-format off
  gl_float_t A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  gl_float_t expected[3*3] = {1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0};
  // clang-format on
  gl_ones(A, 3, 3);
  TEST_ASSERT(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_eye(void) {
  /* Check 4x4 matrix */
  // clang-format off
  gl_float_t A[4*4] = {0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0};
  gl_float_t A_expected[4*4] = {1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
  // clang-format on
  gl_eye(A, 4, 4);
  TEST_ASSERT(gl_equals(A, A_expected, 4, 4, 1e-8));

  /* Check 3x4 matrix */
  // clang-format off
  gl_float_t B[3*4] = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};
  gl_float_t B_expected[3*4] = {1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0,
                             0.0, 0.0, 0.0};
  // clang-format on
  gl_eye(B, 3, 4);
  TEST_ASSERT(gl_equals(B, B_expected, 3, 4, 1e-8));

  return 0;
}

int test_gl_equals(void) {
  // clang-format off
  gl_float_t A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  gl_float_t B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  gl_float_t C[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 10.0};
  // clang-format on

  /* Assert */
  TEST_ASSERT(gl_equals(A, B, 3, 3, 1e-8) == 1);
  TEST_ASSERT(gl_equals(A, C, 3, 3, 1e-8) == 0);

  return 0;
}

int test_gl_mat_set(void) {
  // clang-format off
  gl_float_t A[3*4] = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};
  // clang-format on

  gl_mat_set(A, 3, 4, 0, 1, 1.0);
  gl_mat_set(A, 3, 4, 1, 0, 2.0);
  gl_mat_set(A, 3, 4, 0, 2, 3.0);
  gl_mat_set(A, 3, 4, 2, 0, 4.0);

  return 0;
}

int test_gl_mat_val(void) {
  // clang-format off
  gl_float_t A[3*4] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0};
  // clang-format on

  const float tol = 1e-4;
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 0, 0) - 1.0) < tol);
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 1, 0) - 2.0) < tol);
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 2, 0) - 3.0) < tol);

  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 0, 1) - 4.0) < tol);
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 1, 1) - 5.0) < tol);
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 2, 1) - 6.0) < tol);

  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 0, 2) - 7.0) < tol);
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 1, 2) - 8.0) < tol);
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 2, 2) - 9.0) < tol);

  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 0, 3) - 10.0) < tol);
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 1, 3) - 11.0) < tol);
  TEST_ASSERT(fabs(gl_mat_val(A, 3, 4, 2, 3) - 12.0) < tol);

  return 0;
}

int test_gl_transpose(void) {
  /* Transpose a 3x3 matrix */
  // clang-format off
  gl_float_t A[3*3] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0};
  // clang-format on
  gl_float_t A_t[3 * 3] = {0};

  gl_transpose(A, 3, 3, A_t);

  /* Transpose a 3x4 matrix */
  // clang-format off
  gl_float_t B[3*4] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0};
  // clang-format on
  gl_float_t B_t[3 * 4] = {0};
  gl_transpose(B, 3, 4, B_t);

  return 0;
}

int test_gl_vec3_cross(void) {
  const gl_float_t u[3] = {1.0f, 2.0f, 3.0f};
  const gl_float_t v[3] = {4.0f, 5.0f, 6.0f};
  gl_float_t z[3] = {0};
  gl_vec3_cross(u, v, z);

  /* Assert */
  gl_float_t expected[3] = {-3.0f, 6.0f, -3.0f};
  TEST_ASSERT(gl_equals(z, expected, 3, 1, 1e-8));

  return 0;
}

int test_gl_dot(void) {
  // clang-format off
  gl_float_t A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  gl_float_t B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  // clang-format on
  gl_float_t C[3 * 3] = {0.0};
  gl_dot(A, 3, 3, B, 3, 3, C);

  /* Assert */
  // clang-format off
  gl_float_t expected[3*3] = {30.0f, 66.0f, 102.0f,
                           36.0f, 81.0f, 126.0f,
                           42.0f, 96.0f, 150.0f};
  // clang-format on
  TEST_ASSERT(gl_equals(C, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_norm(void) {
  const gl_float_t x[3] = {1.0f, 2.0f, 3.0f};
  const gl_float_t n = gl_norm(x, 3);

  /* Assert */
  const gl_float_t expected = 3.741657f;
  TEST_ASSERT(fabs(n - expected) < 1e-6);

  return 0;
}

int test_gl_normalize(void) {
  gl_float_t x[3] = {1.0f, 2.0f, 3.0f};
  gl_normalize(x, 3);

  /* Assert */
  const gl_float_t expected[3] = {0.26726f, 0.53452f, 0.80178f};
  TEST_ASSERT(gl_equals(x, expected, 3, 1, 1e-5));

  return 0;
}

int test_gl_perspective(void) {
  const gl_float_t fov = gl_deg2rad(60.0);
  const gl_float_t window_width = 1000.0f;
  const gl_float_t window_height = 1000.0f;
  const gl_float_t ratio = window_width / window_height;
  const gl_float_t near = 0.1f;
  const gl_float_t far = 100.0f;

  gl_float_t P[4 * 4] = {0};
  gl_perspective(fov, ratio, near, far, P);

  // clang-format off
  const gl_float_t P_expected[4*4] = {
    1.732051, 0.000000, 0.000000, 0.000000,
    0.000000, 1.732051, 0.000000, 0.000000,
    0.000000, 0.000000, -1.002002, -1.000000,
    0.000000, 0.000000, -0.200200, 0.000000
  };
  // clang-format on
  TEST_ASSERT(gl_equals(P, P_expected, 4, 4, 1e-4));

  return 0;
}

int test_gl_ortho(void) {
  const gl_float_t left = 0.0f;
  const gl_float_t right = 800.0f;
  const gl_float_t bottom = 0.0f;
  const gl_float_t top = 600.0f;
  const gl_float_t near = -1.0f;
  const gl_float_t far = 1.0f;
  gl_float_t P[4 * 4] = {0};
  gl_ortho(left, right, bottom, top, near, far, P);

  // clang-format off
  const gl_float_t P_expected[4*4] = {
    0.00250000,  0.00000000,  0.000000,  0.000000,
    0.00000000,  0.00333333,  0.000000,  0.000000,
    0.00000000,  0.00000000, -1.000000,  0.000000,
   -1.00000000, -1.00000000,  0.000000,  1.000000
  };
  // clang-format on
  TEST_ASSERT(gl_equals(P, P_expected, 4, 4, 1e-4));

  return 0;
}

int test_gl_lookat(void) {
  const gl_float_t yaw = -0.785398;
  const gl_float_t pitch = 0.000000;
  const gl_float_t radius = 10.000000;
  const gl_float_t focal[3] = {0.000000, 0.000000, 0.000000};
  const gl_float_t world_up[3] = {0.000000, 1.000000, 0.000000};

  gl_float_t eye[3];
  eye[0] = focal[0] + radius * sin(yaw);
  eye[1] = focal[1] + radius * cos(pitch);
  eye[2] = focal[2] + radius * cos(yaw);

  gl_float_t V[4 * 4] = {0};
  gl_lookat(eye, focal, world_up, V);

  // clang-format off
  const gl_float_t V_expected[4*4] = {
    0.707107, 0.500000, -0.500000, 0.000000,
    -0.000000, 0.707107, 0.707107, 0.000000,
    0.707107, -0.500000, 0.500000, 0.000000,
    0.000000, 0.000000, -14.142136, 1.000000
  };
  // clang-format on
  TEST_ASSERT(gl_equals(V, V_expected, 4, 4, 1e-4));

  return 0;
}

// TEST SHADER ///////////////////////////////////////////////////////////////

int test_gl_shader_compile(void) {
  // Setup
  GLFWwindow *window = test_setup();

  // Vertex shader
  char *vs_str = load_file("./shaders/cube.vert");
  if (vs_str == NULL) {
    FATAL("Failed to load file: %s\n", "./shaders/cube.vert");
  }
  const gl_uint_t vs = gl_shader_compile(vs_str, GL_VERTEX_SHADER);
  free(vs_str);
  TEST_ASSERT(vs != GL_FALSE);

  // Fragment shader
  char *fs_str = load_file("./shaders/cube.frag");
  if (fs_str == NULL) {
    FATAL("Failed to load file: %s\n", "./shaders/cube.frag");
  }
  const gl_uint_t fs = gl_shader_compile(fs_str, GL_VERTEX_SHADER);
  free(fs_str);
  TEST_ASSERT(fs != GL_FALSE);

  // Cleanup
  test_teardown(window);

  return 0;
}

int test_gl_shaders_link(void) {
  // Setup
  GLFWwindow *window = test_setup();

  // Cube vertex shader
  char *vs_str = load_file("./shaders/cube.vert");
  const gl_uint_t vs = gl_shader_compile(vs_str, GL_VERTEX_SHADER);
  free(vs_str);
  TEST_ASSERT(vs != GL_FALSE);

  // Cube fragment shader
  char *fs_str = load_file("./shaders/cube.frag");
  const gl_uint_t fs = gl_shader_compile(fs_str, GL_FRAGMENT_SHADER);
  free(fs_str);
  TEST_ASSERT(fs != GL_FALSE);

  // Link shakders
  const gl_uint_t gs = GL_FALSE;
  const gl_uint_t prog = gl_shaders_link(vs, fs, gs);
  TEST_ASSERT(prog != GL_FALSE);

  // Cleanup
  test_teardown(window);

  return 0;
}

// TEST GL PROGRAM ///////////////////////////////////////////////////////////

int test_gl_prog_setup(void) {
  // Setup
  GLFWwindow *window = test_setup();

  // Shader program
  char *vs_str = load_file("./shaders/cube.vert");
  char *fs_str = load_file("./shaders/cube.frag");
  const gl_uint_t program_id = gl_prog_setup(vs_str, fs_str, NULL);
  free(vs_str);
  free(fs_str);
  TEST_ASSERT(program_id != GL_FALSE);

  // Cleanup
  test_teardown(window);

  return 0;
}

// TEST GL-CAMERA ////////////////////////////////////////////////////////////

int test_gl_camera_setup(void) {
  int window_width = 640;
  int window_height = 480;

  gl_camera_t camera;
  gl_camera_setup(&camera, &window_width, &window_height);

  // const gl_float_t focal_expected[3] = {0.0f, 0.0f, 0.0f};
  // const gl_float_t world_up_expected[3] = {0.0f, 1.0f, 0.0f};
  // const gl_float_t position_expected[3] = {0.0f, 2.0f, 0.0f};
  // const gl_float_t right_expected[3] = {-1.0f, 0.0f, 0.0f};
  // const gl_float_t up_expected[3] = {0.0f, 1.0f, 0.0f};
  // const gl_float_t front_expected[3] = {0.0f, 0.0f, 1.0f};
  // const gl_float_t yaw_expected = gl_deg2rad(0.0f);
  // const gl_float_t pitch_expected = gl_deg2rad(0.0f);
  // const gl_float_t fov_expected = gl_deg2rad(90.0f);
  // const gl_float_t near_expected = 0.01f;
  // const gl_float_t far_expected = 100.0f;

  // TEST_ASSERT(camera.window_width == &window_width);
  // TEST_ASSERT(camera.window_height == &window_height);
  //
  // TEST_ASSERT(gl_equals(camera.focal, focal_expected, 3, 1, 1e-8) == 1);
  // TEST_ASSERT(gl_equals(camera.world_up, world_up_expected, 3, 1, 1e-8) == 1);
  // TEST_ASSERT(gl_equals(camera.position, position_expected, 3, 1, 1e-8) == 1);
  // TEST_ASSERT(gl_equals(camera.right, right_expected, 3, 1, 1e-8) == 1);
  // TEST_ASSERT(gl_equals(camera.up, up_expected, 3, 1, 1e-8) == 1);
  // TEST_ASSERT(gl_equals(camera.front, front_expected, 3, 1, 1e-8) == 1);
  // TEST_ASSERT(fabs(camera.yaw - yaw_expected) < 1e-8);
  // TEST_ASSERT(fabs(camera.pitch - pitch_expected) < 1e-8);
  //
  // TEST_ASSERT(fabs(camera.fov - fov_expected) < 1e-8);
  // TEST_ASSERT(fabs(camera.near - near_expected) < 1e-8);
  // TEST_ASSERT(fabs(camera.far - far_expected) < 1e-8);

  return 0;
}

// TEST GL-MODEL /////////////////////////////////////////////////////////////

int test_gl_model_load(void) {
  gl_model_t *model = gl_model_load("/home/chutsu/monkey.obj");
  gl_model_free(model);
  TEST_ASSERT(model == NULL);

  return 0;
}

// TEST GUI //////////////////////////////////////////////////////////////////

int test_gui(void) {
  gui_t gui;
  gui.window_title = "viz";
  gui.window_width = 1024;
  gui.window_height = 768;

  gui_setup(&gui);
  gui_loop(&gui);

  return 0;
}

// TEST SANDBOX //////////////////////////////////////////////////////////////

int test_sandbox(void) {
  GLFWwindow *window = test_setup();

  // // Render loop
  // while (!glfwWindowShouldClose(win)) {
  //   glClear(GL_DEPTH_BUFFER_BIT);
  //   glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  //   glClear(GL_COLOR_BUFFER_BIT);
  //
  //   glfwSwapBuffers(win);
  //   glfwPollEvents();
  // }

  // Clean up
  test_teardown(window);

  return 0;
}

// TEST-SUITE ////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  // TEST(test_glfw);

  TEST(test_gl_zeros);
  TEST(test_gl_ones);
  TEST(test_gl_eye);
  TEST(test_gl_equals);
  TEST(test_gl_mat_set);
  TEST(test_gl_mat_val);
  TEST(test_gl_transpose);
  TEST(test_gl_vec3_cross);
  TEST(test_gl_dot);
  TEST(test_gl_norm);
  TEST(test_gl_normalize);
  TEST(test_gl_perspective);
  TEST(test_gl_ortho);
  TEST(test_gl_lookat);
  TEST(test_gl_shader_compile);
  TEST(test_gl_shaders_link);
  TEST(test_gl_prog_setup);
  // TEST(test_gl_camera_setup);
  // TEST(test_gl_model_load);
  TEST(test_gui);
  TEST(test_sandbox);

  return (num_failed) ? -1 : 0;
}

#endif // GUI_UNITTEST

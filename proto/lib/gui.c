#include "gui.h"

/******************************************************************************
 * UTILS
 ******************************************************************************/

GLfloat gl_deg2rad(const GLfloat d) { return d * M_PI / 180.0f; }

GLfloat gl_rad2deg(const GLfloat r) { return r * 180.0f / M_PI; }

void gl_print_vector(const char *prefix, const GLfloat *x, const int length) {
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
                     const GLfloat *A,
                     const int nb_rows,
                     const int nb_cols) {
  printf("%s:\n", prefix);
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      printf("%f", A[i + (j * nb_rows)]);
      if ((j + 1) != nb_cols) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void gl_zeros(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] = 0.0f;
  }
}

void gl_ones(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] = 1.0f;
  }
}

void gl_eye(GLfloat *A, const int nb_rows, const int nb_cols) {
  int idx = 0;
  for (int j = 0; j < nb_cols; j++) {
    for (int i = 0; i < nb_rows; i++) {
      A[idx++] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

void gl_vec2f(GLfloat *v, const GLfloat x, const GLfloat y) {
  v[0] = x;
  v[1] = y;
}

void gl_vec3f(GLfloat *v, const GLfloat x, const GLfloat y, const GLfloat z) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
}

void gl_vec4f(GLfloat *v,
              const GLfloat x,
              const GLfloat y,
              const GLfloat z,
              const GLfloat w) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
  v[3] = w;
}

int gl_equals(const GLfloat *A,
              const GLfloat *B,
              const int nb_rows,
              const int nb_cols,
              const GLfloat tol) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    if (fabs(A[i] - B[i]) > tol) {
      return 0;
    }
  }

  return 1;
}

void gl_matf_set(GLfloat *A,
                 const int m,
                 const int n,
                 const int i,
                 const int j,
                 const GLfloat val) {
  UNUSED(n);
  A[i + (j * m)] = val;
}

GLfloat gl_matf_val(
    const GLfloat *A, const int m, const int n, const int i, const int j) {
  UNUSED(n);
  return A[i + (j * m)];
}

void gl_copy(const GLfloat *src, const int m, const int n, GLfloat *dest) {
  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

void gl_transpose(const real_t *A, size_t m, size_t n, real_t *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  int idx = 0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A_t[idx++] = gl_matf_val(A, m, n, i, j);
    }
  }
}

void gl_vec3f_cross(const GLfloat u[3], const GLfloat v[3], GLfloat n[3]) {
  assert(u);
  assert(v);
  assert(n);

  n[0] = u[1] * v[2] - u[2] * v[1];
  n[1] = u[2] * v[0] - u[0] * v[2];
  n[2] = u[0] * v[1] - u[1] * v[0];
}

void gl_add(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    C[i] = A[i] + B[i];
  }
}

void gl_sub(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    C[i] = A[i] - B[i];
  }
}

void gl_dot(const GLfloat *A,
            const int A_m,
            const int A_n,
            const GLfloat *B,
            const int B_m,
            const int B_n,
            GLfloat *C) {
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

void gl_scale(GLfloat factor,
              GLfloat *A,
              const int nb_rows,
              const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] *= factor;
  }
}

GLfloat gl_norm(const GLfloat *x, const int size) {
  GLfloat sum_sq = 0.0f;
  for (int i = 0; i < size; i++) {
    sum_sq += x[i] * x[i];
  }

  return sqrt(sum_sq);
}

void gl_normalize(GLfloat *x, const int size) {
  const GLfloat n = gl_norm(x, size);
  for (int i = 0; i < size; i++) {
    x[i] /= n;
  }
}

void gl_perspective(const GLfloat fov,
                    const GLfloat aspect,
                    const GLfloat near,
                    const GLfloat far,
                    GLfloat P[4 * 4]) {
  const GLfloat f = 1.0f / tan(fov * 0.5f);

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

void gl_lookat(const GLfloat eye[3],
               const GLfloat at[3],
               const GLfloat up[3],
               GLfloat V[4 * 4]) {
  /* Z-axis: Camera forward */
  GLfloat z[3] = {0};
  gl_sub(at, eye, 3, 1, z);
  gl_normalize(z, 3);

  /* X-axis: Camera right */
  GLfloat x[3] = {0};
  gl_vec3f_cross(z, up, x);
  gl_normalize(x, 3);

  /* Y-axis: Camera up */
  GLfloat y[3] = {0};
  gl_vec3f_cross(x, z, y);

  /* Negate z-axis */
  gl_scale(-1.0f, z, 3, 1);

  /* Form rotation component */
  GLfloat R[4 * 4] = {0};
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

  /* Form translation component */
  GLfloat T[4 * 4] = {0};
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

  /* Form view matrix */
  gl_zeros(V, 4, 4);
  gl_dot(R, 4, 4, T, 4, 4, V);
}

/******************************************************************************
 * SHADER
 ******************************************************************************/

GLuint shader_compile(const char *shader_src, const int type) {
  if (shader_src == NULL) {
    return GL_FALSE;
  }

  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &shader_src, NULL);
  glCompileShader(shader);

  GLint retval = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &retval);
  if (retval == GL_FALSE) {
    char log[9046] = {0};
    glGetShaderInfoLog(shader, 9046, NULL, log);
    LOG_ERROR("Failed to compile shader:\n%s\n", log);
    return retval;
  }

  return shader;
}

GLuint shaders_link(const GLuint vertex_shader,
                    const GLuint fragment_shader,
                    const GLuint geometry_shader) {
  // Attach shaders to link
  GLuint program = glCreateProgram();
  glAttachShader(program, vertex_shader);
  glAttachShader(program, fragment_shader);
  if (geometry_shader != GL_FALSE) {
    glAttachShader(program, geometry_shader);
  }
  glLinkProgram(program);

  // Link program
  GLint success = 0;
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
 * GL PROGRAM
 ******************************************************************************/

GLuint gl_prog_setup(const char *vs_src,
                     const char *fs_src,
                     const char *gs_src) {
  const GLuint vs = shader_compile(vs_src, GL_VERTEX_SHADER);
  const GLuint fs = shader_compile(fs_src, GL_FRAGMENT_SHADER);
  const GLuint gs = shader_compile(gs_src, GL_GEOMETRY_SHADER);
  const GLuint program_id = shaders_link(vs, fs, gs);
  return program_id;
}

int gl_prog_set_int(const GLint id, const char *k, const GLint v) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1i(location, v);
  return 0;
}

int gl_prog_set_vec2i(const GLint id, const char *k, const GLint v[2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2i(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3i(const GLint id, const char *k, const GLint v[3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3i(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4i(const GLint id, const char *k, const GLint v[4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4i(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_float(const GLint id, const char *k, const GLfloat v) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1f(location, v);
  return 0;
}

int gl_prog_set_vec2f(const GLint id, const char *k, const GLfloat v[2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2f(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3f(const GLint id, const char *k, const GLfloat v[3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3f(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4f(const GLint id, const char *k, const GLfloat v[4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4f(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_mat2f(const GLint id, const char *k, const GLfloat v[2 * 2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix2fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat3f(const GLint id, const char *k, const GLfloat v[3 * 3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix3fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat4f(const GLint id, const char *k, const GLfloat v[4 * 4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix4fv(location, 1, GL_FALSE, v);
  return 0;
}

/******************************************************************************
 * GL-CAMERA
 ******************************************************************************/

void gl_camera_setup(gl_camera_t *camera,
                     int *window_width,
                     int *window_height) {
  camera->window_width = window_width;
  camera->window_height = window_height;

  gl_zeros(camera->focal, 3, 1);
  gl_vec3f(camera->world_up, 0.0f, 1.0f, 0.0f);
  gl_vec3f(camera->position, 0.0f, 0.0f, 0.0f);
  gl_vec3f(camera->right, -1.0f, 0.0f, 0.0f);
  gl_vec3f(camera->up, 0.0f, 1.0f, 0.0f);
  gl_vec3f(camera->front, 0.0f, 0.0f, -1.0f);
  camera->yaw = gl_deg2rad(0.0f);
  camera->pitch = gl_deg2rad(0.0f);
  camera->radius = 10.0f;

  camera->fov = gl_deg2rad(60.0f);
  camera->near = 0.1f;
  camera->far = 100.0f;

  gl_camera_update(camera);
}

void gl_camera_update(gl_camera_t *camera) {
  /* Front vector */
  camera->front[0] = sin(camera->yaw) * cos(camera->pitch);
  camera->front[1] = sin(camera->pitch);
  camera->front[2] = cos(camera->yaw) * cos(camera->pitch);
  gl_normalize(camera->front, 3);

  /* Right vector */
  gl_vec3f_cross(camera->front, camera->world_up, camera->right);
  gl_normalize(camera->right, 3);

  /* Up vector */
  gl_vec3f_cross(camera->right, camera->front, camera->up);
  gl_normalize(camera->up, 3);

  /* Projection matrix */
  const float width = (float) *(camera->window_width);
  const float height = (float) *(camera->window_height);
  const float aspect = width / height;
  gl_perspective(camera->fov, aspect, camera->near, camera->far, camera->P);

  /* View matrix */
  GLfloat eye[3] = {0};
  eye[0] = camera->focal[0] + camera->radius * sin(camera->yaw);
  eye[1] = camera->focal[1] + camera->radius * cos(camera->pitch);
  eye[2] = camera->focal[2] + camera->radius * cos(camera->yaw);
  gl_lookat(eye, camera->focal, camera->world_up, camera->V);
}

void gl_camera_rotate(gl_camera_t *camera,
                      const float factor,
                      const float dx,
                      const float dy) {
  /* Update yaw and pitch */
  float pitch = camera->pitch;
  float yaw = camera->yaw;
  yaw -= dx * factor;
  pitch += dy * factor;

  /* Constrain pitch and yaw */
  pitch = (pitch <= (-M_PI / 2.0) + 1e-5) ? (-M_PI / 2.0) + 1e-5 : pitch;
  pitch = (pitch > 0.0) ? 0.0 : pitch;
  yaw = (yaw > M_PI) ? yaw - 2 * M_PI : yaw;
  yaw = (yaw < -M_PI) ? yaw + 2 * M_PI : yaw;

  /* Update camera attitude */
  camera->pitch = pitch;
  camera->yaw = yaw;
  gl_camera_update(camera);
}

void gl_camera_pan(gl_camera_t *camera,
                   const float factor,
                   const float dx,
                   const float dy) {
  /* camera->focal -= (dy * mouse_sensitivity) * camera->front; */
  /* camera->focal += (dx * mouse_sensitivity) * camera->right; */
  const GLfloat dx_scaled = dx * factor;
  const GLfloat dy_scaled = dy * factor;
  GLfloat front[3] = {camera->front[0], camera->front[1], camera->front[2]};
  GLfloat right[3] = {camera->right[0], camera->right[1], camera->right[2]};
  gl_scale(dy_scaled, front, 3, 1);
  gl_scale(dx_scaled, right, 3, 1);
  gl_sub(camera->focal, front, 3, 1, camera->focal);
  gl_add(camera->focal, right, 3, 1, camera->focal);

  /* limit focal point y-axis */
  camera->focal[1] = (camera->focal[1] < 0) ? 0 : camera->focal[1];
  gl_camera_update(camera);
}

void gl_camera_zoom(gl_camera_t *camera,
                    const float factor,
                    const float dx,
                    const float dy) {
  UNUSED(factor);
  UNUSED(dx);

  if (camera->fov >= gl_deg2rad(0.5f) && camera->fov <= gl_deg2rad(90.0f)) {
    camera->fov -= dy * 0.1;
  }

  if (camera->fov <= gl_deg2rad(0.5f)) {
    camera->fov = gl_deg2rad(5.0f);
  } else if (camera->fov >= gl_deg2rad(90.0f)) {
    camera->fov = gl_deg2rad(90.0f);
  }

  gl_camera_update(camera);
}

/******************************************************************************
 * GUI
 ******************************************************************************/

/* void gui_framebuffer_size_callback(GLFWwindow* window, int width, int height)
 * { */
/*   gui_t *gui = glfwGetWindowUserPointer(window); */
/*   gui->window_width = width; */
/*   gui->window_height = height; */
/*   glViewport(0, 0, gui->window_width, gui->window_height); */
/* } */
/*  */
/* void gui_event_handler(GLFWwindow *window) { */
/*   if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) { */
/*     glfwSetWindowShouldClose(window, 1); */
/*   } */
/*   if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) { */
/*     glfwSetWindowShouldClose(window, 1); */
/*   } */
/* } */
/*  */
/* void gui_error_callback(int error, const char *description) { */
/*   fprintf(stderr, "GLFW Error %d: %s\n", error, description); */
/* } */
/*  */
/* void gui_mouse_cursor_callback(GLFWwindow *window, double x, double y) { */
/*   gui_t *gui = (gui_t *) glfwGetWindowUserPointer(window); */
/*   const float dx = x - gui->last_cursor_x; */
/*   const float dy = y - gui->last_cursor_y; */
/*   gui->last_cursor_x = x; */
/*   gui->last_cursor_y = y; */
/*  */
/*   #<{(| Rotate camera |)}># */
/*   if (gui->left_click) { */
/*     if (gui->last_cursor_set == 0) { */
/*       gui->last_cursor_set = 1; */
/*     } else if (gui->last_cursor_set) { */
/*       gl_camera_rotate(&gui->camera, gui->mouse_sensitivity, dx, dy); */
/*     } */
/*   } */
/*  */
/*   #<{(| Pan camera |)}># */
/*   if (gui->right_click) { */
/*     if (gui->last_cursor_set == 0) { */
/*       gui->last_cursor_set = 1; */
/*     } else if (gui->last_cursor_set) { */
/*       gl_camera_pan(&gui->camera, gui->mouse_sensitivity, dx, dy); */
/*     } */
/*   } */
/*  */
/*   #<{(| Reset cursor |)}># */
/*   if (gui->left_click == 0 && gui->right_click == 0) { */
/*     gui->left_click = 0; */
/*     gui->right_click = 0; */
/*     gui->last_cursor_set = 0; */
/*     gui->last_cursor_x = 0.0; */
/*     gui->last_cursor_y = 0.0; */
/*   } */
/* } */
/*  */
/* void gui_mouse_button_callback(GLFWwindow *window, */
/*                                int btn, */
/*                                int action, */
/*                                int mods) { */
/*   UNUSED(mods); */
/*  */
/*   gui_t *gui = (gui_t *) glfwGetWindowUserPointer(window); */
/*   if (btn == GLFW_MOUSE_BUTTON_LEFT) { */
/*     gui->left_click = (action == GLFW_PRESS) ? 1 : 0; */
/*   } else if (btn == GLFW_MOUSE_BUTTON_RIGHT) { */
/*     gui->right_click = (action == GLFW_PRESS) ? 1 : 0; */
/*   } */
/* } */
/*  */
/* void gui_mouse_scroll_callback(GLFWwindow *window, double dx, double dy) { */
/*   gui_t *gui = (gui_t *) glfwGetWindowUserPointer(window); */
/*   gl_camera_zoom(&gui->camera, gui->mouse_sensitivity, dx, dy); */
/* } */
/*  */
/* void gui_keyboard_callback(GLFWwindow* window, */
/*                                     int key, */
/*                            int scancode, */
/*                            int action, */
/*                            int mods) { */
/*   UNUSED(scancode); */
/*   UNUSED(mods); */
/*  */
/*   gui_t *gui = (gui_t *) glfwGetWindowUserPointer(window); */
/*   if (key == GLFW_KEY_Q && action == GLFW_PRESS) { */
/*     glfwSetWindowShouldClose(gui->window, 1); */
/*   } */
/* } */
/*  */
/* void gui_setup(gui_t *gui) { */
/*   #<{(| GLFW |)}># */
/*   glfwInit(); */
/*   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); */
/*   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3); */
/*   glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); */
/*   gui->window = glfwCreateWindow(gui->window_width, */
/*                                  gui->window_height, */
/*                                  gui->window_title, */
/*                                  NULL, NULL); */
/*   if (gui->window == NULL) { */
/*     glfwTerminate(); */
/*     FATAL("Failed to create GLFW window"); */
/*   } */
/*   glfwMakeContextCurrent(gui->window); */
/*   glfwSetWindowAspectRatio(gui->window, gui->window_width,
 * gui->window_height); */
/*  */
/*   #<{(| Get screen width, height and center the window |)}># */
/*   int xpos = 0; */
/*   int ypos = 0; */
/*   GLFWmonitor *monitor = glfwGetPrimaryMonitor(); */
/*   glfwGetMonitorWorkarea(monitor, */
/*                          &xpos, */
/*                          &ypos, */
/*                          &gui->screen_width, */
/*                          &gui->screen_height); */
/*   int window_x = (gui->screen_width - gui->window_width) / 2.0; */
/*   int window_y = (gui->screen_height - gui->window_height) / 2.0; */
/*   glfwSetWindowPos(gui->window, window_x, window_y); */
/*  */
/*   #<{(| Event handlers |)}># */
/*   glfwSwapInterval(1); */
/*   glfwSetWindowUserPointer(gui->window, gui); */
/*   glfwSetCursorPosCallback(gui->window, gui_mouse_cursor_callback); */
/*   glfwSetMouseButtonCallback(gui->window, gui_mouse_button_callback); */
/*   glfwSetScrollCallback(gui->window, gui_mouse_scroll_callback); */
/*   glfwSetFramebufferSizeCallback(gui->window, gui_framebuffer_size_callback);
 */
/*  */
/*   #<{(| GLEW |)}># */
/*   GLenum err = glewInit(); */
/*   if (err != GLEW_OK) { */
/*     FATAL("glewInit failed: %s", glewGetErrorString(err)); */
/*   } */
/*  */
/*   #<{(| Camera |)}># */
/*   gl_camera_setup(&gui->camera, &gui->window_width, &gui->window_height); */
/*   gui->movement_speed = 50.0f; */
/*   gui->mouse_sensitivity = 0.02f; */
/*  */
/*   #<{(| Cursor |)}># */
/*   gui->left_click = 0; */
/*   gui->right_click = 0; */
/*   gui->last_cursor_set = 0; */
/*   gui->last_cursor_x = 0.0f; */
/*   gui->last_cursor_y = 0.0f; */
/* } */
/*  */
/* void gui_reset(gui_t *gui) { */
/*   #<{(| Camera |)}># */
/*   gui->movement_speed = 50.0f; */
/*   gui->mouse_sensitivity = 0.02f; */
/*  */
/*   #<{(| Cursor |)}># */
/*   gui->left_click = 0; */
/*   gui->right_click = 0; */
/*   gui->last_cursor_set = 0; */
/*   gui->last_cursor_x = 0.0f; */
/*   gui->last_cursor_y = 0.0f; */
/* } */
/*  */
/* void gui_loop(gui_t *gui) { */
/*   gl_entity_t cube; */
/*   GLfloat cube_pos[3] = {0.0, 0.0, 0.0}; */
/*   gl_cube_setup(&cube, cube_pos); */
/*  */
/*   gl_entity_t cube2; */
/*   GLfloat cube2_pos[3] = {2.0, 0.0, 0.0}; */
/*   gl_cube_setup(&cube2, cube2_pos); */
/*  */
/*   gl_entity_t cube3; */
/*   GLfloat cube3_pos[3] = {-2.0, 0.0, 0.0}; */
/*   gl_cube_setup(&cube3, cube3_pos); */
/*  */
/*   gl_entity_t cf; */
/* 	gl_camera_frame_setup(&cf); */
/*  */
/*   gl_entity_t frame; */
/* 	gl_axis_frame_setup(&frame); */
/*  */
/*   gl_entity_t grid; */
/* 	gl_grid_setup(&grid); */
/*  */
/*   while (!glfwWindowShouldClose(gui->window)) { */
/*     gui_event_handler(gui->window); */
/*     glClearColor(0.15f, 0.15f, 0.15f, 1.0f); */
/*     glClear(GL_COLOR_BUFFER_BIT); */
/*  */
/*     #<{(| gl_cube_draw(&cube, &gui->camera); |)}># */
/*     #<{(| gl_cube_draw(&cube2, &gui->camera); |)}># */
/*     #<{(| gl_cube_draw(&cube3, &gui->camera); |)}># */
/*  */
/* 		gl_camera_frame_draw(&cf, &gui->camera); */
/* 		gl_axis_frame_draw(&frame, &gui->camera); */
/* 		gl_grid_draw(&grid, &gui->camera); */
/*  */
/*     glEnable(GL_CULL_FACE); */
/*     glfwSwapBuffers(gui->window); */
/*     glfwPollEvents(); */
/*   } */
/*  */
/* 	gl_cube_cleanup(&cube); */
/* 	gl_cube_cleanup(&cube2); */
/* 	gl_cube_cleanup(&cube3); */
/* 	gl_camera_frame_cleanup(&cf); */
/* 	gl_grid_cleanup(&grid); */
/*  */
/*   glfwTerminate(); */
/* } */

/* GL CUBE ********************************************************************/

void gl_cube_setup(gl_entity_t *entity, GLfloat pos[3]) {
  /* Entity transform */
  gl_eye(entity->T, 4, 4);
  entity->T[12] = pos[0];
  entity->T[13] = pos[1];
  entity->T[14] = pos[2];

  /* Shader program */
  char *glcube_vs = file_read("./shaders/cube.vert");
  char *glcube_fs = file_read("./shaders/cube.frag");
  entity->program_id = gl_prog_setup(glcube_vs, glcube_fs, NULL);
  free(glcube_vs);
  free(glcube_fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Vertices
  // clang-format off
  const float color[3] = {0.9, 0.4, 0.2};
  const float cube_size = 0.5;
  const float r = color[0];
  const float g = color[1];
  const float b = color[2];
  GLfloat vertices[12 * 3 * 6] = {
    // Triangle 1
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 2
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 3
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 4
    cube_size, cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 5
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 6
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 7
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 8
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 9
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 10
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 11
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 12
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b
    // Triangle 12 : end
  };
  const size_t nb_triangles = 12;
  const size_t vertices_per_triangle = 3;
  const size_t nb_vertices = vertices_per_triangle * nb_triangles;
  const size_t vertex_buffer_size = sizeof(float) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size, vertices, GL_STATIC_DRAW);
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
}

void gl_cube_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_cube_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // 12 x 3 indices starting at 0 -> 12 triangles -> 6 squares
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0); // Unbind VAO
}

/* GL CAMERA FRAME ************************************************************/

void gl_camera_frame_setup(gl_entity_t *entity) {
  /* Entity transform */
  gl_eye(entity->T, 4, 4);

  /* Shader program */
  char *vs = file_read("./shaders/camera_frame.vert");
  char *fs = file_read("./shaders/camera_frame.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Form the camera fov frame
  GLfloat fov = gl_deg2rad(60.0);
  GLfloat hfov = fov / 2.0f;
  GLfloat scale = 1.0f;
  GLfloat z = scale;
  GLfloat hwidth = z * tan(hfov);
  const GLfloat lb[3] = {-hwidth, hwidth, z};  // Left bottom
  const GLfloat lt[3] = {-hwidth, -hwidth, z}; // Left top
  const GLfloat rt[3] = {hwidth, -hwidth, z};  // Right top
  const GLfloat rb[3] = {hwidth, hwidth, z};   // Right bottom

  // Rectangle frame
  // clang-format off
  const GLfloat vertices[] = {
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
  const size_t nb_lines = 8;
  const size_t nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(float),
                        (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gl_camera_frame_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_camera_frame_draw(const gl_entity_t *entity,
                          const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // Store original line width
  GLfloat original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  GLfloat line_width = 2.0f;
  glLineWidth(line_width);

  // Draw frame
  const size_t nb_lines = 8;
  const size_t nb_vertices = nb_lines * 2;
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

/* GL AXIS FRAME **************************************************************/

void gl_axis_frame_setup(gl_entity_t *entity) {
  /* Entity transform */
  gl_eye(entity->T, 4, 4);

  /* Shader program */
  char *vs = file_read("./shaders/axis_frame.vert");
  char *fs = file_read("./shaders/axis_frame.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Vertices
  // clang-format off
  static const GLfloat vertices[] = {
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
  const size_t nb_vertices = 6;
  const size_t buffer_size = sizeof(GLfloat) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
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
}

void gl_axis_frame_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_axis_frame_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // Store original line width
  GLfloat original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  GLfloat line_width = 6.0f;
  glLineWidth(line_width);

  // Draw frame
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, 6);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

static GLfloat *glgrid_create_vertices(int grid_size) {
  // Allocate memory for vertices
  int nb_lines = (grid_size + 1) * 2;
  int nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;
  GLfloat *vertices = (GLfloat *) malloc(buffer_size);

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

/* GL GRID ********************************************************************/

void gl_grid_setup(gl_entity_t *entity) {
  /* Entity transform */
  gl_eye(entity->T, 4, 4);

  /* Shader program */
  char *vs = file_read("./shaders/grid.vert");
  char *fs = file_read("./shaders/grid.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Create vertices
  const int grid_size = 10;
  const int nb_lines = (grid_size + 1) * 2;
  const int nb_vertices = nb_lines * 2;
  GLfloat *vertices = glgrid_create_vertices(grid_size);
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(float),
                        (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
  free(vertices);
}

void gl_grid_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_grid_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  const int grid_size = 10;
  const int nb_lines = (grid_size + 1) * 2;
  const int nb_vertices = nb_lines * 2;

  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO
}

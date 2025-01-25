//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef XYZ_GUI_UNITTEST

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
  const gl_float_t w = 800.0f;
  const gl_float_t h = 600.0f;
  gl_float_t P[4 * 4] = {0};
  gl_ortho(w, h, P);

  // clang-format off
  const gl_float_t P_expected[4*4] = {
    0.00250000,  0.00000000,  0.000000,  0.000000,
    0.00000000, -0.00333333,  0.000000,  0.000000,
    0.00000000,  0.00000000, -1.000000,  0.000000,
   -1.00000000,  1.00000000,  0.000000,  1.000000
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

int test_gl_compile(void) {
  // Setup
  GLFWwindow *window = test_setup();

  // Vertex shader
  char *vs_str = load_file("./shaders/cube.vert");
  if (vs_str == NULL) {
    FATAL("Failed to load file: %s\n", "./shaders/cube.vert");
  }
  const gl_uint_t vs = gl_compile(vs_str, GL_VERTEX_SHADER);
  free(vs_str);
  TEST_ASSERT(vs != GL_FALSE);

  // Fragment shader
  char *fs_str = load_file("./shaders/cube.frag");
  if (fs_str == NULL) {
    FATAL("Failed to load file: %s\n", "./shaders/cube.frag");
  }
  const gl_uint_t fs = gl_compile(fs_str, GL_VERTEX_SHADER);
  free(fs_str);
  TEST_ASSERT(fs != GL_FALSE);

  // Cleanup
  test_teardown(window);

  return 0;
}

int test_gl_link(void) {
  // Setup
  GLFWwindow *window = test_setup();

  // Cube vertex shader
  char *vs_str = load_file("./shaders/cube.vert");
  const gl_uint_t vs = gl_compile(vs_str, GL_VERTEX_SHADER);
  free(vs_str);
  TEST_ASSERT(vs != GL_FALSE);

  // Cube fragment shader
  char *fs_str = load_file("./shaders/cube.frag");
  const gl_uint_t fs = gl_compile(fs_str, GL_FRAGMENT_SHADER);
  free(fs_str);
  TEST_ASSERT(fs != GL_FALSE);

  // Link shakders
  const gl_uint_t gs = GL_FALSE;
  const gl_uint_t prog = gl_link(vs, fs, gs);
  TEST_ASSERT(prog != GL_FALSE);

  // Cleanup
  test_teardown(window);

  return 0;
}

// TEST GL PROGRAM ///////////////////////////////////////////////////////////

int test_gl_shader(void) {
  // Setup
  GLFWwindow *window = test_setup();

  // Shader program
  char *vs_str = load_file("./shaders/cube.vert");
  char *fs_str = load_file("./shaders/cube.frag");
  const gl_uint_t program_id = gl_shader(vs_str, fs_str, NULL);
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
  const char *window_title = "viz";
  const int window_width = 1024;
  const int window_height = 768;
  gui_setup(window_title, window_width, window_height);
  gui_loop();

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
  // TEST(test_gl_compile);
  // TEST(test_gl_link);
  // TEST(test_gl_shader);
  // TEST(test_gl_camera_setup);
  // TEST(test_gl_model_load);
  TEST(test_gui);
  TEST(test_sandbox);

  return (num_failed) ? -1 : 0;
}

#endif // XYZ_GUI_UNITTEST

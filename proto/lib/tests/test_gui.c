#include "munit.h"
#include "../gui.h"

/*******************************************************************************
 *                                  UTILS
 ******************************************************************************/

int test_gl_zeros() {
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat expected[3*3] = {0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0};

  gl_zeros(A, 3, 3);
  gl_print_matrix("A", A, 3, 3);
  MU_CHECK(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_ones() {
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat expected[3*3] = {1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0};

  gl_ones(A, 3, 3);
  gl_print_matrix("A", A, 3, 3);
  MU_CHECK(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_eye() {
  /* Check 4x4 matrix */
  GLfloat A[4*4] = {0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0};
  GLfloat A_expected[4*4] = {1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
  gl_eye(A, 4, 4);
  gl_print_matrix("A", A, 4, 4);
  MU_CHECK(gl_equals(A, A_expected, 4, 4, 1e-8));

  /* Check 3x4 matrix */
  GLfloat B[3*4] = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};
  GLfloat B_expected[3*4] = {1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0,
                             0.0, 0.0, 0.0};
  gl_eye(B, 3, 4);
  gl_print_matrix("B", B, 3, 4);
  MU_CHECK(gl_equals(B, B_expected, 3, 4, 1e-8));

  return 0;
}

int test_gl_equals() {
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat C[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 10.0};

  /* Assert */
  MU_CHECK(gl_equals(A, B, 3, 3, 1e-8) == 1);
  MU_CHECK(gl_equals(A, C, 3, 3, 1e-8) == 0);

  return 0;
}

int test_gl_matf_set() {
  GLfloat A[3*4] = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};

  gl_matf_set(A, 3, 4, 0, 1, 1.0);
  gl_matf_set(A, 3, 4, 1, 0, 2.0);
  gl_matf_set(A, 3, 4, 0, 2, 3.0);
  gl_matf_set(A, 3, 4, 2, 0, 4.0);
  gl_print_matrix("A", A, 3, 4);

  return 0;
}

int test_gl_matf_val() {
  GLfloat A[3*4] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0};

  const float tol = 1e-4;
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 0, 0) - 1.0) < tol);
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 1, 0) - 2.0) < tol);
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 2, 0) - 3.0) < tol);

  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 0, 1) - 4.0) < tol);
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 1, 1) - 5.0) < tol);
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 2, 1) - 6.0) < tol);

  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 0, 2) - 7.0) < tol);
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 1, 2) - 8.0) < tol);
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 2, 2) - 9.0) < tol);

  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 0, 3) - 10.0) < tol);
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 1, 3) - 11.0) < tol);
  MU_CHECK(fabs(gl_matf_val(A, 3, 4, 2, 3) - 12.0) < tol);

  return 0;
}

int test_gl_transpose() {
  /* Transpose a 3x3 matrix */
  GLfloat A[3*3] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0};
  GLfloat A_t[3*3] = {0};

  gl_transpose(A, 3, 3, A_t);
  gl_print_matrix("A", A, 3, 3);
  gl_print_matrix("A_t", A_t, 3, 3);

  /* Transpose a 3x4 matrix */
  GLfloat B[3*4] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0};
  GLfloat B_t[3*4] = {0};
  gl_transpose(B, 3, 4, B_t);
  gl_print_matrix("B", B, 3, 4);
  gl_print_matrix("B_t", B_t, 4, 3);

  return 0;
}

int test_gl_vec3_cross() {
  const GLfloat u[3] = {1.0f, 2.0f, 3.0f};
  const GLfloat v[3] = {4.0f, 5.0f, 6.0f};
  GLfloat z[3] = {0};
  gl_vec3f_cross(u, v, z);

  /* Assert */
  GLfloat expected[3] = {-3.0f, 6.0f, -3.0f};
  gl_print_vector("z", z, 3);
  gl_print_vector("expected", z, 3);
  MU_CHECK(gl_equals(z, expected, 3, 1, 1e-8));

  return 0;
}

int test_gl_dot() {
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat C[3*3] = {0.0};
  gl_dot(A, 3, 3, B, 3, 3, C);

  /* Assert */
  GLfloat expected[3*3] = {30.0f, 66.0f, 102.0f,
                           36.0f, 81.0f, 126.0f,
                           42.0f, 96.0f, 150.0f};
  gl_print_matrix("C", C, 3, 3);
  gl_print_matrix("expected", expected, 3, 3);
  MU_CHECK(gl_equals(C, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_norm() {
  const GLfloat x[3] = {1.0f, 2.0f, 3.0f};
  const GLfloat n = gl_norm(x, 3);

  /* Assert */
  const GLfloat expected = 3.741657f;
  MU_CHECK(fabs(n - expected) < 1e-6);

  return 0;
}

int test_gl_normalize() {
  GLfloat x[3] = {1.0f, 2.0f, 3.0f};
  gl_normalize(x, 3);

  /* Assert */
  const GLfloat expected[3] = {0.26726f, 0.53452f, 0.80178f};
  MU_CHECK(gl_equals(x, expected, 3, 1, 1e-5));

  return 0;
}

int test_gl_perspective() {
  const GLfloat fov = gl_deg2rad(60.0);
  const GLfloat window_width = 1000.0f;
  const GLfloat window_height = 1000.0f;
  const GLfloat ratio = window_width / window_height;
  const GLfloat near = 0.1f;
  const GLfloat far = 100.0f;

  GLfloat P[4*4] = {0};
  gl_perspective(fov, ratio, near, far, P);

  /* clang-format off */
  const GLfloat P_expected[4*4] = {1.886051, 0.000000, 0.000000, 0.000000,
                                   0.000000, 1.732051, 0.000000, 0.000000,
                                   0.000000, 0.000000, -1.002002, -1.000000,
                                   0.000000, 0.000000, -0.200200, 0.000000};
  /* clang-format on */
  printf("fov: %f\n", fov);
  printf("ratio: %f\n", ratio);
  printf("near: %f\n", near);
  printf("far: %f\n", far);
  printf("\n");
  gl_print_matrix("P", P, 4, 4);
  gl_print_matrix("P_expected", P_expected, 4, 4);
  MU_CHECK(gl_equals(P, P_expected, 4, 4, 1e-4));

  return 0;
}

int test_gl_lookat() {
  const GLfloat yaw = -0.785398;
  const GLfloat pitch = 0.000000;
  const GLfloat radius = 10.000000;
  const GLfloat focal[3] = {0.000000, 0.000000, 0.000000};
  const GLfloat world_up[3] = {0.000000, 1.000000, 0.000000};

  GLfloat eye[3];
  eye[0] = focal[0] + radius * sin(yaw);
  eye[1] = focal[1] + radius * cos(pitch);
  eye[2] = focal[2] + radius * cos(yaw);

  GLfloat V[4*4] = {0};
  gl_lookat(eye, focal, world_up, V);

  /* clang-format off */
  const GLfloat V_expected[4*4] = {0.707107, 0.500000, -0.500000, 0.000000,
                                   -0.000000, 0.707107, 0.707107, 0.000000,
                                   0.707107, -0.500000, 0.500000, 0.000000,
                                   0.000000, 0.000000, -14.142136, 1.000000};
  /* clang-format on */
  /* gl_print_vector("eye", eye, 3); */
  /* gl_print_vector("focal", focal, 3); */
  /* gl_print_vector("world_up", world_up, 3); */
  /* printf("\n"); */
  /* gl_print_matrix("V", V, 4, 4); */
  /* gl_print_matrix("V_expected", V_expected, 4, 4); */
  MU_CHECK(gl_equals(V, V_expected, 4, 4, 1e-4));

  return 0;
}

/*******************************************************************************
 *                                  SHADER
 ******************************************************************************/

int test_shader_compile() {
  /* #<{(| GLFW |)}># */
  /* glfwInit(); */
  /* glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); */
  /* glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3); */
  /* glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); */
  /* glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); */
  /* GLFWwindow *window = glfwCreateWindow(1, 1, "Test", NULL, NULL); */
  /* glfwMakeContextCurrent(window); */
  /*  */
  /* #<{(| GLEW |)}># */
  /* GLenum err = glewInit(); */
  /* if (err != GLEW_OK) { */
  /*   printf("glewInit failed: %s", glewGetErrorString(err)); */
  /*   exit(1); */
  /* } */

  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0){
    printf("SDL_Init Error: %s/n", SDL_GetError());
    return -1;
  }

	/* Window */
	const char *title = "Hello World!";
	const int x = 100;
	const int y = 100;
	const int w = 640;
	const int h = 480;
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
	SDL_Window *window = SDL_CreateWindow(title, x, y, w, h, SDL_WINDOW_OPENGL);
	if (window == NULL){
		printf("SDL_CreateWindow Error: %s/n", SDL_GetError());
		return -1;
	}

	/* OpenGL context */
  SDL_GLContext context = SDL_GL_CreateContext(window);
	SDL_GL_SetSwapInterval(1);
	UNUSED(context);

	/* GLEW */
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    FATAL("glewInit failed: %s", glewGetErrorString(err));
  }

  char *glcube_vs = file_read("./shaders/cube.vert");
  const GLuint vs = shader_compile(glcube_vs, GL_VERTEX_SHADER);
  free(glcube_vs);
  MU_CHECK(vs != GL_FALSE);

  char *glcube_fs = file_read("./shaders/cube.frag");
  const GLuint fs = shader_compile(glcube_fs, GL_VERTEX_SHADER);
  free(glcube_fs);
  MU_CHECK(fs != GL_FALSE);

  return 0;
}

int test_shader_link() {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0){
    printf("SDL_Init Error: %s/n", SDL_GetError());
    return -1;
  }

	/* Window */
	const char *title = "Hello World!";
	const int x = 100;
	const int y = 100;
	const int w = 640;
	const int h = 480;
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
	SDL_Window *window = SDL_CreateWindow(title, x, y, w, h, SDL_WINDOW_OPENGL);
	if (window == NULL){
		printf("SDL_CreateWindow Error: %s/n", SDL_GetError());
		return -1;
	}

	/* OpenGL context */
  SDL_GLContext context = SDL_GL_CreateContext(window);
	SDL_GL_SetSwapInterval(1);
	UNUSED(context);

	/* GLEW */
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    FATAL("glewInit failed: %s", glewGetErrorString(err));
  }

  /* Cube vertex shader */
  char *glcube_vs = file_read("./shaders/cube.vert");
  const GLuint vs = shader_compile(glcube_vs, GL_VERTEX_SHADER);
  free(glcube_vs);
  MU_CHECK(vs != GL_FALSE);

  /* Cube fragment shader */
  char *glcube_fs = file_read("./shaders/cube.frag");
  const GLuint fs = shader_compile(glcube_fs, GL_FRAGMENT_SHADER);
  free(glcube_fs);
  MU_CHECK(fs != GL_FALSE);

  /* Link shakders */
  const GLuint gs = GL_FALSE;
  const GLuint prog = shaders_link(vs, fs, gs);
  MU_CHECK(prog != GL_FALSE);

  return 0;
}

/*******************************************************************************
 *                                GL PROGRAM
 ******************************************************************************/

/* int test_gl_prog_setup() { */
/*   #<{(| GLFW |)}># */
/*   glfwInit(); */
/*   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); */
/*   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3); */
/*   glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); */
/*   glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); */
/*   GLFWwindow *window = glfwCreateWindow(1, 1, "Test", NULL, NULL); */
/*   glfwMakeContextCurrent(window); */
/*  */
/*   #<{(| GLEW |)}># */
/*   GLenum err = glewInit(); */
/*   if (err != GLEW_OK) { */
/*     printf("glewInit failed: %s", glewGetErrorString(err)); */
/*     exit(1); */
/*   } */
/*  */
/*   #<{(| Shader program |)}># */
/*   char *glcube_vs = file_read("./shaders/cube.vert"); */
/*   char *glcube_fs = file_read("./shaders/cube.frag"); */
/*   const GLuint program_id = gl_prog_setup(glcube_vs, glcube_fs, NULL); */
/*   free(glcube_vs); */
/*   free(glcube_fs); */
/*   MU_CHECK(program_id != GL_FALSE); */
/*  */
/*   return 0; */
/* } */

/*******************************************************************************
 *                                 GL-CAMERA
 ******************************************************************************/

int test_gl_camera_setup() {
  int window_width = 640;
  int window_height = 480;

  gl_camera_t camera;
  gl_camera_setup(&camera, &window_width, &window_height);

  const GLfloat focal_expected[3] = {0.0f, 0.0f, 0.0f};
  const GLfloat world_up_expected[3] = {0.0f, 1.0f, 0.0f};
  const GLfloat position_expected[3] = {0.0f, 0.0f, 0.0f};
  const GLfloat right_expected[3] = {-1.0f, 0.0f, 0.0f};
  const GLfloat up_expected[3] = {0.0f, 1.0f, 0.0f};
  const GLfloat front_expected[3] = {0.0f, 0.0f, -1.0f};
  const GLfloat yaw_expected = gl_deg2rad(0.0f);
  const GLfloat pitch_expected = gl_deg2rad(0.0f);
  const GLfloat fov_expected = gl_deg2rad(45.0f);
  const GLfloat near_expected = 0.1f;
  const GLfloat far_expected = 100.0f;

  MU_CHECK(camera.window_width == &window_width);
  MU_CHECK(camera.window_height == &window_height);

  MU_CHECK(gl_equals(camera.focal, focal_expected, 3, 1, 1e-8) == 1);
  MU_CHECK(gl_equals(camera.world_up, world_up_expected, 3, 1, 1e-8) == 1);
  MU_CHECK(gl_equals(camera.position, position_expected, 3, 1, 1e-8) == 1);
  MU_CHECK(gl_equals(camera.right, right_expected, 3, 1, 1e-8) == 1);
  MU_CHECK(gl_equals(camera.up, up_expected, 3, 1, 1e-8) == 1);
  MU_CHECK(gl_equals(camera.front, front_expected, 3, 1, 1e-8) == 1);
  MU_CHECK(fabs(camera.yaw - yaw_expected) < 1e-8);
  MU_CHECK(fabs(camera.pitch - pitch_expected) < 1e-8);

  MU_CHECK(fabs(camera.fov - fov_expected) < 1e-8);
  MU_CHECK(fabs(camera.near - near_expected) < 1e-8);
  MU_CHECK(fabs(camera.far - far_expected) < 1e-8);

  return 0;
}

/*******************************************************************************
 *                                   GUI
 ******************************************************************************/

/* int test_gui() { */
/*   gui_t gui; */
/*   gui.window_title = "Test"; */
/*   gui.window_width = 640; */
/*   gui.window_height = 480; */
/*   gui_setup(&gui); */
/*   gui_loop(&gui); */
/*  */
/*   return 0; */
/* } */

int test_gui() {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0){
    printf("SDL_Init Error: %s/n", SDL_GetError());
    return -1;
  }

	/* Window */
	const char *title = "Hello World!";
	const int x = 100;
	const int y = 100;
	const int w = 640;
	const int h = 480;
	const uint32_t flags = SDL_WINDOW_SHOWN;
	SDL_Window *window = SDL_CreateWindow(title, x, y, w, h, flags);
	if (window == NULL){
		printf("SDL_CreateWindow Error: %s/n", SDL_GetError());
		return -1;
	}

	/* OpenGL context */
  SDL_GLContext context = SDL_GL_CreateContext(window);
	UNUSED(context);

	int loop = 1;
	while (loop) {
		glViewport(0, 0, w, h);
    glClearColor(1.f, 0.f, 1.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT);
    SDL_GL_SwapWindow(window);
	}

  // Surface
  /* SDL_Surface *surface = SDL_GetWindowSurface(window); */
  /* SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0xFF, 0xFF, 0xFF)); */
  /* SDL_UpdateWindowSurface(window); */

	SDL_Delay(2000);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}

void test_suite() {
  /* UTILS */
  MU_ADD_TEST(test_gl_zeros);
  MU_ADD_TEST(test_gl_ones);
  MU_ADD_TEST(test_gl_eye);
  MU_ADD_TEST(test_gl_equals);
  MU_ADD_TEST(test_gl_matf_set);
  MU_ADD_TEST(test_gl_matf_val);
  MU_ADD_TEST(test_gl_transpose);
  MU_ADD_TEST(test_gl_vec3_cross);
  MU_ADD_TEST(test_gl_dot);
  MU_ADD_TEST(test_gl_norm);
  MU_ADD_TEST(test_gl_normalize);
  MU_ADD_TEST(test_gl_perspective);
  MU_ADD_TEST(test_gl_lookat);

  /* SHADER */
  MU_ADD_TEST(test_shader_compile);
  MU_ADD_TEST(test_shader_link);

  /* GL PROGRAM */
  /* MU_ADD_TEST(test_gl_prog_setup); */

  /* GL CAMERA */
  MU_ADD_TEST(test_gl_camera_setup);

  /* GUI */
  /* MU_ADD_TEST(test_gui); */
}

MU_RUN_TESTS(test_suite)

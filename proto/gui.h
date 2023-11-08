#ifndef GUI_H
#define GUI_H

#include <stdio.h>
#include <assert.h>
#include <libgen.h>

#include "stb_image.h"

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#define SDL_DISABLE_IMMINTRIN_H 1
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>

/******************************************************************************
 * OPENGL UTILS
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
GLfloat gl_matf_val(const GLfloat *A,
                    const int m,
                    const int n,
                    const int i,
                    const int j);
void gl_copy(const GLfloat *src, const int m, const int n, GLfloat *dest);
void gl_transpose(const GLfloat *A, size_t m, size_t n, GLfloat *A_t);
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
int gl_save_frame_buffer(const int width, const int height, const char *fp);

/******************************************************************************
 * SHADER
 *****************************************************************************/

GLuint gl_shader_compile(const char *shader_src, const int type);
GLuint gl_shaders_link(const GLuint vertex_shader,
                       const GLuint fragment_shader,
                       const GLuint geometry_shader);

/******************************************************************************
 * GL-PROGRAM
 *****************************************************************************/

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
 *****************************************************************************/

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
 * GL-PRIMITIVES
 *****************************************************************************/

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

  unsigned int VAO;
  unsigned int VBO;
  unsigned int EBO;
} gl_mesh_t;

void gl_mesh_setup(gl_mesh_t *mesh,
                   gl_vertex_t *vertices,
                   const int num_vertices,
                   unsigned int *indices,
                   const int num_indices,
                   gl_texture_t *textures,
                   const int num_textures) {
  mesh->vertices = vertices;
  mesh->num_vertices = num_vertices;
  mesh->indices = indices;
  mesh->num_indices = num_indices;
  mesh->textures = textures;
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

  glBindVertexArray(0);
}

void gl_mesh_draw(const gl_mesh_t *mesh, const GLuint shader) {
  // bind appropriate textures
  unsigned int num_diffuse = 1;
  unsigned int num_specular = 1;
  unsigned int num_normal = 1;
  unsigned int num_height = 1;

  for (int i = 0; i < mesh->num_textures; i++) {
    // Active proper texture unit before binding
    glActiveTexture(GL_TEXTURE0 + i);

    // Form texture unit (the N in diffuse_textureN)
    char texture_unit[30] = {0};
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

typedef struct gl_model_t {
  char *model_dir;

  gl_texture_t *textures;
  gl_mesh_t *meshes;
  int num_textures;
  int num_meshes;

  int enable_gamma_correction;
} gl_model_t;

static unsigned int load_texture(const char *model_dir,
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
  int width = 0;
  int height = 0;
  int channels = 0;
  unsigned char *data = stbi_load(filepath, &width, &height, &channels, 0);
  if (data) {
    // Image format
    GLenum format;
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
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 format,
                 width,
                 height,
                 0,
                 format,
                 GL_UNSIGNED_BYTE,
                 data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,
                    GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  } else {
    printf("Texture failed to load: [%s]\n", filepath);
    return -1;
  }

  // Clean up
  stbi_image_free(data);

  return texture_id;
}

static void load_textures(const struct aiMaterial *material,
                          const enum aiTextureType type,
                          const char *model_dir,
                          gl_texture_t *textures,
                          int *textures_length) {
  const int num_textures = aiGetMaterialTextureCount(material, type);
  int texture_index = *textures_length - 1;

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
  }

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

    textures[texture_index].id = load_texture(model_dir, texture_fname.data);
    strcpy(textures[texture_index].type, type_name);
    strcpy(textures[texture_index].path, texture_fname.data);
    texture_index++;
    (*textures_length)++;
  }
}

static gl_mesh_t *load_mesh(const char *model_dir,
                            struct aiMesh *mesh,
                            const struct aiScene *scene) {
  // For each mesh vertices
  const int num_vertices = mesh->mNumVertices;
  gl_vertex_t *vertices = malloc(sizeof(gl_vertex_t) * num_vertices);
  for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
    // Vertex position
    vertices[i].position[0] = mesh->mVertices[i].x;
    vertices[i].position[1] = mesh->mVertices[i].y;
    vertices[i].position[2] = mesh->mVertices[i].z;

    // Vertex normal
    if (mesh->mNormals != NULL && mesh->mNumVertices > 0) {
      vertices[i].normal[0] = mesh->mNormals[i].x;
      vertices[i].normal[1] = mesh->mNormals[i].y;
      vertices[i].normal[2] = mesh->mNormals[i].z;
    }

    // Vertex texture coordinates
    if (mesh->mTextureCoords[0]) {
      // Texture coordinates
      vertices[i].tex_coords[0] = mesh->mTextureCoords[0][i].x;
      vertices[i].tex_coords[1] = mesh->mTextureCoords[0][i].y;
      // Note: A vertex can contain up to 8 different texture coordinates. We
      // thus make the assumption that we won't use models where a vertex can
      // have multiple texture coordinates so we always take the first set (0).

      // Tangent
      vertices[i].tangent[0] = mesh->mTangents[i].x;
      vertices[i].tangent[1] = mesh->mTangents[i].y;
      vertices[i].tangent[2] = mesh->mTangents[i].z;

      // Bitangent
      vertices[i].bitangent[0] = mesh->mBitangents[i].x;
      vertices[i].bitangent[1] = mesh->mBitangents[i].y;
      vertices[i].bitangent[2] = mesh->mBitangents[i].z;

    } else {
      // Default Texture coordinates
      vertices[i].tex_coords[0] = 0.0f;
      vertices[i].tex_coords[1] = 0.0f;
    }
  }

  // For each mesh face
  // -- Determine number of indices
  int num_indices = 0;
  for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
    for (unsigned int j = 0; j < mesh->mFaces[i].mNumIndices; j++) {
      num_indices++;
    }
  }
  // -- Form indices array
  unsigned int *indices = malloc(sizeof(unsigned int) * num_indices);
  int index_counter = 0;
  for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
    for (unsigned int j = 0; j < mesh->mFaces[i].mNumIndices; j++) {
      indices[index_counter] = mesh->mFaces[i].mIndices[j];
      index_counter++;
    }
  }

  // Process texture materials
  struct aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
  // Note: we assume a convention for sampler names in the shaders. Each diffuse texture should be named
  // as 'texture_diffuseN' where N is a sequential number ranging from 1 to MAX_SAMPLER_NUMBER.
  // Same applies to other texture as the following list summarizes:
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
  int textures_length = 0;
  gl_texture_t *textures = malloc(sizeof(gl_texture_t) * num_textures);

  load_textures(material,
                aiTextureType_DIFFUSE,
                model_dir,
                textures,
                &textures_length);
  load_textures(material,
                aiTextureType_SPECULAR,
                model_dir,
                textures,
                &textures_length);
  load_textures(material,
                aiTextureType_HEIGHT,
                model_dir,
                textures,
                &textures_length);
  load_textures(material,
                aiTextureType_AMBIENT,
                model_dir,
                textures,
                &textures_length);

  // Form Mesh
  gl_mesh_t *gl_mesh = malloc(sizeof(gl_mesh_t));
  gl_mesh_setup(gl_mesh,
                vertices,
                num_vertices,
                indices,
                num_indices,
                textures,
                num_textures);

  return gl_mesh;
}

static void process_node(const char *model_dir,
                         const struct aiNode *node,
                         const struct aiScene *scene) {
  // Process each mesh located at the current node
  for (unsigned int i = 0; i < node->mNumMeshes; i++) {
    // The node object only contains indices to index the actual objects in the
    // scene. The scene contains all the data, node is just to keep stuff
    // organized (like relations between nodes).
    load_mesh(model_dir, scene->mMeshes[node->mMeshes[i]], scene);
  }

  // After processing all of the meshes (if any) we then recursively process
  // each of the children nodes
  for (unsigned int i = 0; i < node->mNumChildren; i++) {
    process_node(model_dir, node->mChildren[i], scene);
  }
}

int gl_model_setup(gl_model_t *model, const char *model_path) {
  // Using assimp to load model
  const struct aiScene *scene =
      aiImportFile(model_path, aiProcessPreset_TargetRealtime_MaxQuality);
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE ||
      !scene->mRootNode) {
    printf("Failed to load model: %s\n", model_path);
    return -1;
  }

  // Get model directory
  char path[9046] = {0};
  strcpy(path, model_path);
  model->model_dir = dirname(path);
  if (model->model_dir == NULL) {
    printf("Failed to get directory name of [%s]!", model_path);
    return -1;
  }

  // Process model
  process_node(model->model_dir, scene->mRootNode, scene);

  return 0;
}

void gl_model_draw(const gl_model_t *model, const GLuint shader) {
  for (int i = 0; i < model->num_meshes; i++) {
    gl_mesh_draw(&model->meshes[i], shader);
  }
}

/******************************************************************************
 * GUI
 *****************************************************************************/

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

/******************************************************************************
 * IMSHOW
 *****************************************************************************/

typedef struct imshow_t {
  SDL_Window *window;
  SDL_Renderer *renderer;

  char *window_title;
  int window_width;
  int window_height;
  int loop;

  SDL_Surface *image_surface;

  gl_camera_t camera;
  GLfloat movement_speed;
  GLfloat mouse_sensitivity;

  int left_click;
  int right_click;
  int last_cursor_set;
  float last_cursor_x;
  float last_cursor_y;
} imshow_t;

void imshow_window_callback(imshow_t *imshow, const SDL_Event event);
void imshow_keyboard_callback(imshow_t *imshow, const SDL_Event event);
void imshow_event_handler(imshow_t *gui);
void imshow_setup(imshow_t *imshow, const char *fp);
void imshow_reset(imshow_t *imshow);
void imshow_loop(imshow_t *imshow);
#endif // GUI_H

#ifdef GUI_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#include <time.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/**
 * Tic, start timer.
 * @returns A timespec encapsulating the time instance when tic() is called
 */
struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

/**
 * Toc, stop timer.
 * @returns Time elapsed in seconds
 */
float toc(struct timespec *tic) {
  assert(tic != NULL);
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

/******************************************************************************
 * OPENGL UTILS
 *****************************************************************************/

/**
 * Read file contents in file path `fp`.
 * @returns
 * - Success: File contents
 * - Failure: NULL
 */
static char *file_read(const char *fp) {
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

GLfloat gl_deg2rad(const GLfloat d) {
  return d * M_PI / 180.0f;
}

GLfloat gl_rad2deg(const GLfloat r) {
  return r * 180.0f / M_PI;
}

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

GLfloat gl_matf_val(const GLfloat *A,
                    const int m,
                    const int n,
                    const int i,
                    const int j) {
  UNUSED(n);
  return A[i + (j * m)];
}

void gl_copy(const GLfloat *src, const int m, const int n, GLfloat *dest) {
  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

void gl_transpose(const GLfloat *A, size_t m, size_t n, GLfloat *A_t) {
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
  // Z-axis: Camera forward
  GLfloat z[3] = {0};
  gl_sub(at, eye, 3, 1, z);
  gl_normalize(z, 3);

  // X-axis: Camera right
  GLfloat x[3] = {0};
  gl_vec3f_cross(z, up, x);
  gl_normalize(x, 3);

  // Y-axis: Camera up
  GLfloat y[3] = {0};
  gl_vec3f_cross(x, z, y);

  // Negate z-axis
  gl_scale(-1.0f, z, 3, 1);

  // Form rotation component
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

  // Form translation component
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
  const GLint x = 0;
  const GLint y = 0;
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
 * SHADER
 *****************************************************************************/

GLuint gl_shader_compile(const char *shader_src, const int type) {
  if (shader_src == NULL) {
    LOG_ERROR("Shader source is NULL!");
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
    LOG_ERROR("Failed to compile shader:\n%s", log);
    return retval;
  }

  return shader;
}

GLuint gl_shaders_link(const GLuint vertex_shader,
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
 * GL-PROGRAM
 *****************************************************************************/

GLuint gl_prog_setup(const char *vs_src,
                     const char *fs_src,
                     const char *gs_src) {
  GLuint vs = GL_FALSE;
  GLuint fs = GL_FALSE;
  GLuint gs = GL_FALSE;

  if (vs_src) {
    vs = gl_shader_compile(vs_src, GL_VERTEX_SHADER);
  }

  if (fs_src) {
    fs = gl_shader_compile(fs_src, GL_FRAGMENT_SHADER);
  }

  if (gs_src) {
    gs = gl_shader_compile(gs_src, GL_GEOMETRY_SHADER);
  }

  const GLuint program_id = gl_shaders_link(vs, fs, gs);
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
 *****************************************************************************/

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
  // Front vector
  camera->front[0] = sin(camera->yaw) * cos(camera->pitch);
  camera->front[1] = sin(camera->pitch);
  camera->front[2] = cos(camera->yaw) * cos(camera->pitch);
  gl_normalize(camera->front, 3);

  // Right vector
  gl_vec3f_cross(camera->front, camera->world_up, camera->right);
  gl_normalize(camera->right, 3);

  // Up vector
  gl_vec3f_cross(camera->right, camera->front, camera->up);
  gl_normalize(camera->up, 3);

  // Projection matrix
  const float width = (float) *(camera->window_width);
  const float height = (float) *(camera->window_height);
  const float aspect = width / height;
  gl_perspective(camera->fov, aspect, camera->near, camera->far, camera->P);

  // View matrix
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
  // Update yaw and pitch
  float pitch = camera->pitch;
  float yaw = camera->yaw;
  yaw -= dx * factor;
  pitch += dy * factor;

  // Constrain pitch and yaw
  pitch = (pitch <= (-M_PI / 2.0) + 1e-5) ? (-M_PI / 2.0) + 1e-5 : pitch;
  pitch = (pitch > 0.0) ? 0.0 : pitch;
  yaw = (yaw > M_PI) ? yaw - 2 * M_PI : yaw;
  yaw = (yaw < -M_PI) ? yaw + 2 * M_PI : yaw;

  // Update camera attitude
  camera->pitch = pitch;
  camera->yaw = yaw;
  gl_camera_update(camera);
}

void gl_camera_pan(gl_camera_t *camera,
                   const float factor,
                   const float dx,
                   const float dy) {
  // camera->focal -= (dy * mouse_sensitivity) * camera->front;
  // camera->focal += (dx * mouse_sensitivity) * camera->right;
  const GLfloat dx_scaled = dx * factor;
  const GLfloat dy_scaled = dy * factor;
  GLfloat front[3] = {camera->front[0], camera->front[1], camera->front[2]};
  GLfloat right[3] = {camera->right[0], camera->right[1], camera->right[2]};
  gl_scale(dy_scaled, front, 3, 1);
  gl_scale(dx_scaled, right, 3, 1);
  gl_sub(camera->focal, front, 3, 1, camera->focal);
  gl_add(camera->focal, right, 3, 1, camera->focal);

  // limit focal point y-axis
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
 * GL-PRIMITIVES
 *****************************************************************************/

void gl_cube_setup(gl_entity_t *entity, GLfloat pos[3]) {
  // Entity transform
  gl_eye(entity->T, 4, 4);
  entity->T[12] = pos[0];
  entity->T[13] = pos[1];
  entity->T[14] = pos[2];

  // Shader program
  char *vs = file_read("./shaders/cube.vert");
  char *fs = file_read("./shaders/cube.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
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

// GL CAMERA FRAME ///////////////////////////////////////////////////////////

void gl_camera_frame_setup(gl_entity_t *entity) {
  // Entity transform
  gl_eye(entity->T, 4, 4);

  // Shader program
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

// GL AXIS FRAME /////////////////////////////////////////////////////////////

void gl_axis_frame_setup(gl_entity_t *entity) {
  // Entity transform
  gl_eye(entity->T, 4, 4);

  // Shader program
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

// GL GRID ///////////////////////////////////////////////////////////////////

void gl_grid_setup(gl_entity_t *entity) {
  // Entity transform
  gl_eye(entity->T, 4, 4);

  // Shader program
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

/******************************************************************************
 * GUI
 *****************************************************************************/

void gui_window_callback(gui_t *gui, const SDL_Event event) {
  if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
    const int width = event.window.data1;
    const int height = event.window.data2;
    gui->screen_width = width;
    gui->screen_height = (height > 0) ? height : 1;
    glViewport(0, 0, (GLsizei) gui->screen_width, (GLsizei) gui->screen_height);
  }
}

void gui_keyboard_callback(gui_t *gui, const SDL_Event event) {
  if (event.type == SDL_KEYDOWN) {
    switch (event.key.keysym.sym) {
      case SDLK_ESCAPE:
        gui->loop = 0;
        break;
      case SDLK_q:
        gui->loop = 0;
        break;
    }
  }
}

void gui_mouse_callback(gui_t *gui, const SDL_Event event) {
  const float x = event.motion.x;
  const float y = event.motion.y;
  const float dx = x - gui->last_cursor_x;
  const float dy = y - gui->last_cursor_y;
  gui->last_cursor_x = x;
  gui->last_cursor_y = y;

  gui->left_click = (event.button.button == SDL_BUTTON_LEFT);
  gui->right_click = (event.button.button == SDL_BUTTON_RIGHT ||
                      event.button.button == SDL_BUTTON_X1);

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
  } else if (event.wheel.type == SDL_MOUSEWHEEL && event.wheel.y) {
    gl_camera_zoom(&gui->camera, gui->mouse_sensitivity, 0, event.wheel.y);
  } else {
    // Reset cursor
    gui->left_click = 0;
    gui->right_click = 0;
    gui->last_cursor_set = 0;
    gui->last_cursor_x = 0.0;
    gui->last_cursor_y = 0.0;
  }
}

void gui_event_handler(gui_t *gui) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_WINDOWEVENT:
        gui_window_callback(gui, event);
        break;
      case SDL_KEYUP:
      case SDL_KEYDOWN:
        gui_keyboard_callback(gui, event);
        break;
      case SDL_MOUSEMOTION:
      case SDL_MOUSEBUTTONDOWN:
      case SDL_MOUSEBUTTONUP:
      case SDL_MOUSEWHEEL:
        gui_mouse_callback(gui, event);
        break;
    }
  }
}

void gui_setup(gui_t *gui) {
  // SDL init
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    FATAL("SDL_Init Error: %s/n", SDL_GetError());
  }

  // Get display size
  SDL_DisplayMode disp_mode;
  SDL_GetCurrentDisplayMode(0, &disp_mode);
  const int disp_w = disp_mode.w;
  const int disp_h = disp_mode.h;

  // Window
  const char *title = "Hello World!";
  const int w = 640;
  const int h = 480;
  const int x = disp_w / 2 - w / 2;
  const int y = disp_h / 2 - h / 2;
  const uint32_t flags = SDL_WINDOW_OPENGL;
  gui->window = SDL_CreateWindow(title, x, y, w, h, flags);
  if (gui->window == NULL) {
    FATAL("SDL_CreateWindow Error: %s/n", SDL_GetError());
  }
  SDL_SetWindowResizable(gui->window, 0);

  // OpenGL context
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_GLContext context = SDL_GL_CreateContext(gui->window);
  SDL_GL_SetSwapInterval(1);
  UNUSED(context);

  // GLEW
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    FATAL("glewInit failed: %s", glewGetErrorString(err));
  }

  // Camera
  gl_camera_setup(&gui->camera, &gui->window_width, &gui->window_height);
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
  gl_entity_t cube;
  GLfloat cube_pos[3] = {0.0, 0.0, 0.0};
  gl_cube_setup(&cube, cube_pos);

  gl_entity_t cube2;
  GLfloat cube2_pos[3] = {2.0, 0.0, 0.0};
  gl_cube_setup(&cube2, cube2_pos);

  gl_entity_t cube3;
  GLfloat cube3_pos[3] = {-2.0, 0.0, 0.0};
  gl_cube_setup(&cube3, cube3_pos);

  gl_entity_t cf;
  gl_camera_frame_setup(&cf);

  gl_entity_t frame;
  gl_axis_frame_setup(&frame);

  gl_entity_t grid;
  gl_grid_setup(&grid);

  gui->loop = 1;
  while (gui->loop) {
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // gl_cube_draw(&cube, &gui->camera);
    // gl_cube_draw(&cube2, &gui->camera);
    // gl_cube_draw(&cube3, &gui->camera);

    gl_camera_frame_draw(&cf, &gui->camera);
    gl_axis_frame_draw(&frame, &gui->camera);
    gl_grid_draw(&grid, &gui->camera);

    // int width, height;
    // SDL_GetWindowSize(gui->window, &width, &height);
    // gl_save_frame_buffer(width, height, "/tmp/frame.png");

    gui_event_handler(gui);
    SDL_GL_SwapWindow(gui->window);
    SDL_Delay(1);
  }

  gl_cube_cleanup(&cube);
  gl_cube_cleanup(&cube2);
  gl_cube_cleanup(&cube3);
  gl_camera_frame_cleanup(&cf);
  gl_grid_cleanup(&grid);

  SDL_DestroyWindow(gui->window);
  SDL_Quit();
}

/******************************************************************************
 * IMSHOW
 *****************************************************************************/

void imshow_window_callback(imshow_t *imshow, const SDL_Event event) {
  if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
    SDL_Surface *screen_surface = SDL_GetWindowSurface(imshow->window);

    Uint32 color = SDL_MapRGB(screen_surface->format, 0, 0, 0);
    SDL_FillRect(screen_surface, NULL, color);

    SDL_Rect stretch;
    stretch.x = 0;
    stretch.y = 0;
    stretch.w = event.window.data1;
    stretch.h = event.window.data2;
    SDL_BlitScaled(imshow->image_surface, NULL, screen_surface, &stretch);
  }
}

void imshow_keyboard_callback(imshow_t *imshow, const SDL_Event event) {
  if (event.type == SDL_KEYDOWN) {
    switch (event.key.keysym.sym) {
      case SDLK_ESCAPE:
        imshow->loop = 0;
        break;
      case SDLK_q:
        imshow->loop = 0;
        break;
    }
  }
}

void imshow_event_handler(imshow_t *imshow) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_WINDOWEVENT:
        imshow_window_callback(imshow, event);
        break;
      case SDL_KEYUP:
      case SDL_KEYDOWN:
        imshow_keyboard_callback(imshow, event);
        break;
    }
  }
}

void draw_circle(SDL_Renderer *renderer,
                 const int cx,
                 const int cy,
                 const int radius,
                 const SDL_Color color) {
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
  for (int x = cx - radius; x <= cx + radius; x++) {
    for (int y = cy - radius; y <= cy + radius; y++) {
      if ((pow(cy - y, 2) + pow(cx - x, 2)) <= pow(radius, 2)) {
        SDL_RenderDrawPoint(renderer, x, y);
      }
    }
  }
}

// void draw_circle(SDL_Renderer *renderer,
//                  int32_t centreX,
//                  int32_t centreY,
//                  int32_t radius,
//                  const SDL_Color color) {
//   SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
//   const int32_t diameter = (radius * 2);
//
//   int32_t x = (radius - 1);
//   int32_t y = 0;
//   int32_t tx = 1;
//   int32_t ty = 1;
//   int32_t error = (tx - diameter);
//
//   while (x >= y) {
//     //  Each of the following renders an octant of the circle
//     SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
//     SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
//     SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
//     SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
//     SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
//     SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
//     SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
//     SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);
//
//     if (error <= 0) {
//       ++y;
//       error += ty;
//       ty += 2;
//     }
//
//     if (error > 0) {
//       --x;
//       tx += 2;
//       error += (tx - diameter);
//     }
//   }
// }

void imshow_setup(imshow_t *im, const char *fp) {
  // SDL init
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    FATAL("SDL_Init Error: %s/n", SDL_GetError());
  }

  // Load image
  im->image_surface = IMG_Load(fp);
  if (im->image_surface == NULL) {
    FATAL("Failed to load image [%s]!", fp);
  }
  const int img_w = im->image_surface->w;
  const int img_h = im->image_surface->h;

  // Get display size
  SDL_DisplayMode disp_mode;
  SDL_GetCurrentDisplayMode(0, &disp_mode);
  const int disp_w = disp_mode.w;
  const int disp_h = disp_mode.h;

  // Create window
  const int x = disp_w / 2 - img_w / 2;
  const int y = disp_h / 2 - img_h / 2;
  const int w = img_w;
  const int h = img_h;
  if (SDL_CreateWindowAndRenderer(w, h, 0, &im->window, &im->renderer) != 0) {
    FATAL("Failed to create window: %s\n", SDL_GetError());
  }
  SDL_SetWindowTitle(im->window, im->window_title);
  SDL_SetWindowPosition(im->window, x, y);
  SDL_SetWindowResizable(im->window, 1);

  // Clear render
  SDL_SetRenderDrawColor(im->renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(im->renderer);

  // Show image
  SDL_Texture *texture =
      SDL_CreateTextureFromSurface(im->renderer, im->image_surface);
  SDL_RenderCopy(im->renderer, texture, NULL, NULL);
  SDL_RenderPresent(im->renderer);

  // Draw circles
  const int x_min = 0;
  const int y_min = 0;
  const int x_max = img_w;
  const int y_max = img_h;
  for (int i = 0; i < 200; i++) {
    const int x = (rand() % (x_max + 1 - x_min)) + x_min;
    const int y = (rand() % (y_max + 1 - y_min)) + y_min;
    const int radius = 5;
    SDL_Color color;
    color.r = 255;
    color.g = 0;
    color.b = 0;
    color.a = 255;
    draw_circle(im->renderer, x, y, radius, color);
  }
  SDL_RenderPresent(im->renderer);

  // Cursor
  im->left_click = 0;
  im->right_click = 0;
  im->last_cursor_set = 0;
  im->last_cursor_x = 0.0f;
  im->last_cursor_y = 0.0f;
}

void imshow_reset(imshow_t *imshow) {
  // Camera
  imshow->movement_speed = 50.0f;
  imshow->mouse_sensitivity = 0.02f;

  // Cursor
  imshow->left_click = 0;
  imshow->right_click = 0;
  imshow->last_cursor_set = 0;
  imshow->last_cursor_x = 0.0f;
  imshow->last_cursor_y = 0.0f;
}

void imshow_loop(imshow_t *imshow) {
  imshow->loop = 1;
  while (imshow->loop) {
    imshow_event_handler(imshow);
    SDL_UpdateWindowSurface(imshow->window);
    SDL_Delay(1);
  }

  SDL_DestroyWindow(imshow->window);
  SDL_Quit();
}

#endif // GUI_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef GUI_UNITTEST

#include <stdio.h>

// UNITESTS GLOBAL VARIABLES
static int nb_tests = 0;
static int nb_passed = 0;
static int nb_failed = 0;

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
void run_test(const char *test_name, int (*test_ptr)()) {
  if ((*test_ptr)() == 0) {
    printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
    fflush(stdout);
    nb_passed++;
  } else {
    printf(TERM_RED "FAILED!\n" TERM_NRM);
    fflush(stdout);
    nb_failed++;
  }
  nb_tests++;
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

// TEST OPENGL UTILS /////////////////////////////////////////////////////////

int test_gl_zeros() {
  // clang-format off
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat expected[3*3] = {0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0};
  // clang-format on

  gl_zeros(A, 3, 3);
  gl_print_matrix("A", A, 3, 3);
  TEST_ASSERT(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_ones() {
  // clang-format off
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat expected[3*3] = {1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0};
  // clang-format on

  gl_ones(A, 3, 3);
  gl_print_matrix("A", A, 3, 3);
  TEST_ASSERT(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_eye() {
  /* Check 4x4 matrix */
  // clang-format off
  GLfloat A[4*4] = {0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0};
  GLfloat A_expected[4*4] = {1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
  // clang-format on
  gl_eye(A, 4, 4);
  gl_print_matrix("A", A, 4, 4);
  TEST_ASSERT(gl_equals(A, A_expected, 4, 4, 1e-8));

  /* Check 3x4 matrix */
  // clang-format off
  GLfloat B[3*4] = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};
  GLfloat B_expected[3*4] = {1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0,
                             0.0, 0.0, 0.0};
  // clang-format on
  gl_eye(B, 3, 4);
  gl_print_matrix("B", B, 3, 4);
  TEST_ASSERT(gl_equals(B, B_expected, 3, 4, 1e-8));

  return 0;
}

int test_gl_equals() {
  // clang-format off
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat C[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 10.0};
  // clang-format on

  /* Assert */
  TEST_ASSERT(gl_equals(A, B, 3, 3, 1e-8) == 1);
  TEST_ASSERT(gl_equals(A, C, 3, 3, 1e-8) == 0);

  return 0;
}

int test_gl_matf_set() {
  // clang-format off
  GLfloat A[3*4] = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};
  // clang-format on

  gl_matf_set(A, 3, 4, 0, 1, 1.0);
  gl_matf_set(A, 3, 4, 1, 0, 2.0);
  gl_matf_set(A, 3, 4, 0, 2, 3.0);
  gl_matf_set(A, 3, 4, 2, 0, 4.0);
  gl_print_matrix("A", A, 3, 4);

  return 0;
}

int test_gl_matf_val() {
  // clang-format off
  GLfloat A[3*4] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0};
  // clang-format on

  const float tol = 1e-4;
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 0, 0) - 1.0) < tol);
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 1, 0) - 2.0) < tol);
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 2, 0) - 3.0) < tol);

  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 0, 1) - 4.0) < tol);
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 1, 1) - 5.0) < tol);
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 2, 1) - 6.0) < tol);

  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 0, 2) - 7.0) < tol);
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 1, 2) - 8.0) < tol);
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 2, 2) - 9.0) < tol);

  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 0, 3) - 10.0) < tol);
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 1, 3) - 11.0) < tol);
  TEST_ASSERT(fabs(gl_matf_val(A, 3, 4, 2, 3) - 12.0) < tol);

  return 0;
}

int test_gl_transpose() {
  /* Transpose a 3x3 matrix */
  // clang-format off
  GLfloat A[3*3] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0};
  // clang-format on
  GLfloat A_t[3 * 3] = {0};

  gl_transpose(A, 3, 3, A_t);
  gl_print_matrix("A", A, 3, 3);
  gl_print_matrix("A_t", A_t, 3, 3);

  /* Transpose a 3x4 matrix */
  // clang-format off
  GLfloat B[3*4] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0};
  // clang-format on
  GLfloat B_t[3 * 4] = {0};
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
  TEST_ASSERT(gl_equals(z, expected, 3, 1, 1e-8));

  return 0;
}

int test_gl_dot() {
  // clang-format off
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  // clang-format on
  GLfloat C[3 * 3] = {0.0};
  gl_dot(A, 3, 3, B, 3, 3, C);

  /* Assert */
  // clang-format off
  GLfloat expected[3*3] = {30.0f, 66.0f, 102.0f,
                           36.0f, 81.0f, 126.0f,
                           42.0f, 96.0f, 150.0f};
  // clang-format on
  gl_print_matrix("C", C, 3, 3);
  gl_print_matrix("expected", expected, 3, 3);
  TEST_ASSERT(gl_equals(C, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_norm() {
  const GLfloat x[3] = {1.0f, 2.0f, 3.0f};
  const GLfloat n = gl_norm(x, 3);

  /* Assert */
  const GLfloat expected = 3.741657f;
  TEST_ASSERT(fabs(n - expected) < 1e-6);

  return 0;
}

int test_gl_normalize() {
  GLfloat x[3] = {1.0f, 2.0f, 3.0f};
  gl_normalize(x, 3);

  /* Assert */
  const GLfloat expected[3] = {0.26726f, 0.53452f, 0.80178f};
  TEST_ASSERT(gl_equals(x, expected, 3, 1, 1e-5));

  return 0;
}

int test_gl_perspective() {
  const GLfloat fov = gl_deg2rad(60.0);
  const GLfloat window_width = 1000.0f;
  const GLfloat window_height = 1000.0f;
  const GLfloat ratio = window_width / window_height;
  const GLfloat near = 0.1f;
  const GLfloat far = 100.0f;

  GLfloat P[4 * 4] = {0};
  gl_perspective(fov, ratio, near, far, P);

  // clang-format off
  const GLfloat P_expected[4*4] = {1.886051, 0.000000, 0.000000, 0.000000,
                                   0.000000, 1.732051, 0.000000, 0.000000,
                                   0.000000, 0.000000, -1.002002, -1.000000,
                                   0.000000, 0.000000, -0.200200, 0.000000};
  // clang-format on
  printf("fov: %f\n", fov);
  printf("ratio: %f\n", ratio);
  printf("near: %f\n", near);
  printf("far: %f\n", far);
  printf("\n");
  gl_print_matrix("P", P, 4, 4);
  gl_print_matrix("P_expected", P_expected, 4, 4);
  TEST_ASSERT(gl_equals(P, P_expected, 4, 4, 1e-4));

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

  GLfloat V[4 * 4] = {0};
  gl_lookat(eye, focal, world_up, V);

  // clang-format off
  const GLfloat V_expected[4*4] = {0.707107, 0.500000, -0.500000, 0.000000,
                                   -0.000000, 0.707107, 0.707107, 0.000000,
                                   0.707107, -0.500000, 0.500000, 0.000000,
                                   0.000000, 0.000000, -14.142136, 1.000000};
  // clang-format on
  /* gl_print_vector("eye", eye, 3); */
  /* gl_print_vector("focal", focal, 3); */
  /* gl_print_vector("world_up", world_up, 3); */
  /* printf("\n"); */
  /* gl_print_matrix("V", V, 4, 4); */
  /* gl_print_matrix("V_expected", V_expected, 4, 4); */
  TEST_ASSERT(gl_equals(V, V_expected, 4, 4, 1e-4));

  return 0;
}

// TEST SHADER ///////////////////////////////////////////////////////////////

int test_gl_shader_compile() {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
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
  if (window == NULL) {
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
  const GLuint vs = gl_shader_compile(glcube_vs, GL_VERTEX_SHADER);
  free(glcube_vs);
  TEST_ASSERT(vs != GL_FALSE);

  char *glcube_fs = file_read("./shaders/cube.frag");
  const GLuint fs = gl_shader_compile(glcube_fs, GL_VERTEX_SHADER);
  free(glcube_fs);
  TEST_ASSERT(fs != GL_FALSE);

  return 0;
}

int test_gl_shaders_link() {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
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
  if (window == NULL) {
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
  const GLuint vs = gl_shader_compile(glcube_vs, GL_VERTEX_SHADER);
  free(glcube_vs);
  TEST_ASSERT(vs != GL_FALSE);

  /* Cube fragment shader */
  char *glcube_fs = file_read("./shaders/cube.frag");
  const GLuint fs = gl_shader_compile(glcube_fs, GL_FRAGMENT_SHADER);
  free(glcube_fs);
  TEST_ASSERT(fs != GL_FALSE);

  /* Link shakders */
  const GLuint gs = GL_FALSE;
  const GLuint prog = gl_shaders_link(vs, fs, gs);
  TEST_ASSERT(prog != GL_FALSE);

  return 0;
}

// TEST GL PROGRAM ///////////////////////////////////////////////////////////

int test_gl_prog_setup() {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
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
  if (window == NULL) {
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

  /* Shader program */
  char *glcube_vs = file_read("./shaders/cube.vert");
  char *glcube_fs = file_read("./shaders/cube.frag");
  const GLuint program_id = gl_prog_setup(glcube_vs, glcube_fs, NULL);
  free(glcube_vs);
  free(glcube_fs);
  TEST_ASSERT(program_id != GL_FALSE);

  return 0;
}

// TEST GL-CAMERA ////////////////////////////////////////////////////////////

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

  TEST_ASSERT(camera.window_width == &window_width);
  TEST_ASSERT(camera.window_height == &window_height);

  TEST_ASSERT(gl_equals(camera.focal, focal_expected, 3, 1, 1e-8) == 1);
  TEST_ASSERT(gl_equals(camera.world_up, world_up_expected, 3, 1, 1e-8) == 1);
  TEST_ASSERT(gl_equals(camera.position, position_expected, 3, 1, 1e-8) == 1);
  TEST_ASSERT(gl_equals(camera.right, right_expected, 3, 1, 1e-8) == 1);
  TEST_ASSERT(gl_equals(camera.up, up_expected, 3, 1, 1e-8) == 1);
  TEST_ASSERT(gl_equals(camera.front, front_expected, 3, 1, 1e-8) == 1);
  TEST_ASSERT(fabs(camera.yaw - yaw_expected) < 1e-8);
  TEST_ASSERT(fabs(camera.pitch - pitch_expected) < 1e-8);

  TEST_ASSERT(fabs(camera.fov - fov_expected) < 1e-8);
  TEST_ASSERT(fabs(camera.near - near_expected) < 1e-8);
  TEST_ASSERT(fabs(camera.far - far_expected) < 1e-8);

  return 0;
}

// TEST GUI //////////////////////////////////////////////////////////////////

int test_gui() {
  gui_t gui;
  gui.window_title = "Test";
  gui.window_width = 640;
  gui.window_height = 480;

  gui_setup(&gui);
  gui_loop(&gui);

  return 0;
}

// TEST IMSHOW ///////////////////////////////////////////////////////////////

int test_imshow() {
  imshow_t imshow;
  imshow.window_title = "Test";
  imshow_setup(&imshow, "test_data/images/awesomeface.png");

  /* int x = 100; */
  /* int y = 100; */
  /* int radius = 10; */
  /* SDL_Color color; */
  /* color.r = 0; */
  /* color.g = 0; */
  /* color.b = 0; */
  /* color.a = 255; */
  /* draw_circle(imshow.renderer, x, y, radius, color); */

  imshow_loop(&imshow);

  return 0;
}

int main(int argc, char *argv[]) {
  // TEST(test_gl_zeros);
  // TEST(test_gl_ones);
  // TEST(test_gl_eye);
  // TEST(test_gl_equals);
  // TEST(test_gl_matf_set);
  // TEST(test_gl_matf_val);
  // TEST(test_gl_transpose);
  // TEST(test_gl_vec3_cross);
  // TEST(test_gl_dot);
  // TEST(test_gl_norm);
  // TEST(test_gl_normalize);
  // TEST(test_gl_perspective);
  // TEST(test_gl_lookat);
  // TEST(test_gl_shader_compile);
  // TEST(test_gl_shaders_link);
  // TEST(test_gl_prog_setup);
  // TEST(test_gl_camera_setup);
  TEST(test_gui);
  // TEST(test_imshow);

  return (nb_failed) ? -1 : 0;
}

#endif // GUI_UNITTEST

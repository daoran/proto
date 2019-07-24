#include "play/shader.hpp"

int shader_compile(const char *shader_src, const int type) {
	assert(shader_src != nullptr);

  int shader = glCreateShader(type);
  glShaderSource(shader, 1, &shader_src, NULL);
  glCompileShader(shader);

  int success = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char log[512];
    glGetShaderInfoLog(shader, 512, NULL, log);
    printf("Failed to compile fragment shader:\n%s\n", log);
    return -1;
  }

  return shader;
}

int shader_compile(const std::string &shader_path, const int type) {
  // Open shader file
  std::ifstream shader_file{shader_path};
  if (shader_file.good() == false) {
    return -1;
  }

  // Parse file to string
  std::stringstream ss;
  ss << shader_file.rdbuf();
  shader_file.close();
  const std::string str = ss.str();
  const char *shader_src = str.c_str();

  return shader_compile(shader_src, type);
}

int shaders_link(const int vertex_shader,
                 const int fragment_shader,
                 const int geometry_shader) {
  assert(vertex_shader != -1);
  assert(fragment_shader != -1);

  // Attach shaders to link
  int program = glCreateProgram();
  glAttachShader(program, vertex_shader);
  glAttachShader(program, fragment_shader);
  if (geometry_shader != -1) {
    glAttachShader(program, geometry_shader);
  }
  glLinkProgram(program);

  // Link program
  int success = 0;
  char log[1024];
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(program, 1024, NULL, log);
    printf("Failed to link shaders:\n%s\n", log);
    exit(-1);
  }

  // Delete shaders
  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);
  if (geometry_shader == -1) {
    glDeleteShader(geometry_shader);
  }

  return program;
}

glprog_t::glprog_t(const std::string &vs_path,
                   const std::string &fs_path) {
  const int vs = shader_compile(vs_path, GL_VERTEX_SHADER);
  const int fs = shader_compile(fs_path, GL_FRAGMENT_SHADER);
    program_id = shaders_link(vs, fs);
}

glprog_t::glprog_t(const std::string &vs_path,
                   const std::string &fs_path,
                   const std::string &gs_path) {
  const int vs = shader_compile(vs_path, GL_VERTEX_SHADER);
  const int fs = shader_compile(fs_path, GL_FRAGMENT_SHADER);
  const int gs = shader_compile(gs_path, GL_GEOMETRY_SHADER);
  program_id = shaders_link(vs, fs, gs);
}

glprog_t::glprog_t(const char *vs_src, const char *fs_src) {
  assert(vs_src != nullptr);
  assert(fs_src != nullptr);
  const int vs = shader_compile(vs_src, GL_VERTEX_SHADER);
  const int fs = shader_compile(fs_src, GL_FRAGMENT_SHADER);
  program_id = shaders_link(vs, fs);
}

void glprog_t::use() const {
  glUseProgram(program_id);
}

int glprog_t::setBool(const std::string &key, const bool value) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform1i(location, (int) value);
	return 0;
}

int glprog_t::setInt(const std::string &key, const int value) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform1i(location, value);
}

int glprog_t::setFloat(const std::string &key, const float value) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform1f(location, value);
}

int glprog_t::setVec2(const std::string &key,
                      const glm::vec2 &value) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform2fv(location, 1, &value[0]);
}

int glprog_t::setVec2(const std::string &key,
                      const float x, const float y) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform2f(location, x, y);
}

int glprog_t::setVec3(const std::string &key,
                      const glm::vec3 &value) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform3fv(location, 1, &value[0]);
}

int glprog_t::setVec3(const std::string &key,
                      const float x,
                      const float y,
                      const float z) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform3f(location, x, y, z);
}

int glprog_t::setVec4(const std::string &key,
                      const glm::vec4 &value) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform4fv(location, 1, &value[0]);
}

int glprog_t::setVec4(const std::string &key,
                      const float x,
                      const float y,
                      const float z,
                      const float w) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniform4f(location, x, y, z, w);
}

int glprog_t::setMat2(const std::string &key, const glm::mat2 &mat) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniformMatrix2fv(location, 1, GL_FALSE, &mat[0][0]);
}

int glprog_t::setMat3(const std::string &key, const glm::mat3 &mat) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniformMatrix3fv(location, 1, GL_FALSE, &mat[0][0]);
}

int glprog_t::setMat4(const std::string &key, const glm::mat4 &mat) const {
  const auto location = glGetUniformLocation(program_id, key.c_str());
	if (location == -1) {
		return -1;
	}

  glUniformMatrix4fv(location, 1, GL_FALSE, &mat[0][0]);
}

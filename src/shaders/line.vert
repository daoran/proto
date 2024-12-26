#version 330 core
layout (location = 0) in vec3 in_pos;
out vec3 color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 in_color;

void main() {
  gl_Position = projection * view * model * vec4(in_pos, 1.0);
  color = in_color;
}

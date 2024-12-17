#version 330 core
// in vec2 tex_coord;
out vec4 frag_color;
// uniform sampler2D tex_data;

void main() {
  // frag_color = texture(tex_data, tex_coord);
  frag_color = vec4(0.5f, 0.5f, 01.0f, 1.0f);
}

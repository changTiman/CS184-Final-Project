#version 330

uniform vec4 u_color;

in vec4 v_position;

out vec4 out_color;

vec3 white = vec3(1, 1, 1);

void main() {
  out_color = vec4(white, 1.0);
}
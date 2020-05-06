#version 330

uniform vec4 u_color;

in vec4 v_position;
in vec4 v_normal;
in float v_temp;

out vec4 out_color;

vec3 white = vec3(1, 1, 1);
vec3 blue = vec3(0, 0, 1);

void main() {
  //out_color = u_color;

  float a = (200 - v_temp) / 200;

  out_color = vec4(mix(white, blue, a), 1.0);
}

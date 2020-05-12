#version 330

uniform vec4 u_color;

in vec4 v_position;
in float v_vel;

out vec4 out_color;

vec3 white = vec3(1, 1, 1);
vec3 black = vec3(0, 0, 0);

void main() {
// need to change this to match with source velocity
  float a = (5 - v_vel) / 5;

  out_color = vec4(mix(white, black, a), 1.0);
}
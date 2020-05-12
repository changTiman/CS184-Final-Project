#version 330

uniform vec4 u_color;

in vec4 v_position;
in float v_vel;

out vec4 out_color;

vec3 red = vec3(1, 0, 0);
vec3 blue = vec3(0, 0, 1);

void main() {
// need to change this to match with source velocity
  float a = (5 - v_vel) / 5;

  out_color = vec4(mix(red, blue, a), 1.0);
}
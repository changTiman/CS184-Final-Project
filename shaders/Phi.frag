#version 330

uniform vec4 u_color;

in vec4 v_position;
in vec4 v_normal;
in float v_phi;
in float v_temp;

out vec4 out_color;

vec3 white = vec3(1, 1, 1);
vec3 blue = vec3(0, 0, 1);
vec3 red = vec3(1, 0, 0);

void main() {
  //out_color = u_color;

  if (v_phi > 2) {
      out_color = vec4(red, 1.0);
  } else if (v_phi > 0) {
	  out_color = vec4(blue, 1.0);
  } else {
	  out_color = vec4(white, 1.0);
  }
}

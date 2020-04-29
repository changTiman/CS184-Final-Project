#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 r = u_light_pos - vec3(v_position);
  
  float n_dot_l = dot(vec3(v_normal), r);
  vec3 diffuse = vec3(.4, .4, .4) * u_light_intensity / dot(r, r) * max(0, n_dot_l);
  //vec3 diffuse = vec3(0, 0, 0);
  
  vec3 h = normalize((u_cam_pos - vec3(v_position)) + r);
  float n_dot_h = dot(vec3(v_normal), h);
  vec3 specular = vec3(1, 1, 1) * u_light_intensity / dot(r, r) * pow(max(0, n_dot_h), 128);
  //vec3 specular = vec3(0, 0, 0);

  vec3 ambient = vec3(1, 1, 1) * vec3(.1, .1, .1);
  //vec3 ambient = vec3(0, 0, 0);

  out_color = vec4(ambient + diffuse + specular, 1);
}


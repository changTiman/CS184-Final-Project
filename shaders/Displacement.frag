#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return 2.0 * texture(u_texture_2, uv).rgb - 1.0;
}

void main() {
  // YOUR CODE HERE
  vec3 b = normalize(cross(vec3(v_normal), vec3(v_tangent)));
  mat3 TBN = mat3(vec3(v_tangent), b, vec3(v_normal));
  
  float u_offset = v_uv.x + 1 / u_texture_2_size.x;
  float v_offset = v_uv.y + 1 / u_texture_2_size.y;

  float dU = (h(v_uv) - h(vec2(u_offset, v_uv.y))) * u_height_scaling * u_normal_scaling;
  float dV = (h(v_uv) - h(vec2(v_uv.x, v_offset))) * u_height_scaling * u_normal_scaling;

  vec3 no = normalize(vec3(dU, dV, 1));
  
  vec3 nd = normalize(TBN * no);

  //out_color = vec4(nd, 1);

  vec3 r = u_light_pos - vec3(v_position);
  
  float n_dot_l = dot(nd, r);
  vec3 diffuse = vec3(.4, .4, .4) * u_light_intensity / dot(r, r) * max(0, n_dot_l);

  vec3 h = normalize((u_cam_pos - vec3(v_position)) + r);
  float n_dot_h = dot(nd, h);
  vec3 specular = vec3(1, 1, 1) * u_light_intensity / dot(r, r) * pow(max(0, n_dot_h), 128);

  vec3 ambient = vec3(1, 1, 1) * vec3(.1, .1, .1);

  // seems kinda dim not sure if need to copy phong code into here
  // but without phong shading code then the normals are colorful and still faint
  out_color = vec4(ambient + diffuse + specular, 1);
}


#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 wo = u_cam_pos - vec3(v_position);
  vec3 wi = reflect(wo, vec3(v_normal));

  vec4 sample = texture(u_texture_cubemap, wi);

  out_color = sample;
  out_color.a = 1;
}

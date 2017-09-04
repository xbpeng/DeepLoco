#version 130

in vec3 inPosition;
in vec3 inNormal;
in vec2 inTexCoord;

varying		vec2		tex_coord;
 
void main(void) {
  gl_Position = vec4(inPosition.xy, 0.0, 1.0);
  tex_coord = (inPosition.xy + 1.0) / 2.0;
}
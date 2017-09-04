#version 130

uniform		sampler2D	gBufferTex;
varying		vec2		tex_coord;

void main(void) {
	gl_FragColor = texture2D( gBufferTex, tex_coord.xy );
}
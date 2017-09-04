#version 130

uniform mat4 gl_ModelViewMatrix;
uniform mat4 gl_ProjectionMatrix;

in vec3 inPosition;
in vec3 inNormal;
in vec2 inTexCoord;

varying vec3 ViewPos;
varying vec3 Normal;
varying	vec4 VertColor;
varying vec2 TexCoord;

void main()
{
	vec4 view_pos = gl_ModelViewMatrix * vec4(inPosition.xyz, 1.f);
    vec4 WVP_Pos = gl_ProjectionMatrix * view_pos;

    gl_Position = WVP_Pos;
	ViewPos = view_pos.xyz;
	Normal = (gl_ModelViewMatrix * vec4(inNormal.xyz, 0.f)).xyz;
	VertColor = gl_Color;
	TexCoord = inTexCoord.xy;
}

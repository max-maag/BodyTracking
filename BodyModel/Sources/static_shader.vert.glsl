#version 450

in vec3 pos;
in vec4 col;
in vec3 nor;

in mat4 M;
in mat4 MInverse;

out vec3 eyeCoord;
out vec3 diffuseCol;
out vec3 normal;

uniform mat4 P;
uniform mat4 V;

void kore() {
	// Pass some variables to the fragment shader
	eyeCoord = (M * vec4(pos, 1.0)).xyz;
	diffuseCol = col.xyz;
	normal = normalize((transpose(mat4(MInverse)) * vec4(nor, 0.0)).xyz);
	
	// Apply all matrix transformations to vert
	gl_Position = P * V * M * vec4(pos, 1.0);
}
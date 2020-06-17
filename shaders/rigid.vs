#version 430
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;
uniform mat4 normalMat;
uniform bool showTexture;
out vec3 vNormal;
out vec3 vPosW;
out vec2 vTexCoords;
void main()
{
	vNormal.xyz = (normalize(normalMat * vec4(aNormal, 1.0))).xyz;
	vPosW.xyz = (model * vec4(aPosition, 1.0)).xyz;
	vTexCoords = showTexture ? vTexCoords = aTexCoords : vec2(0.0);
	gl_Position = proj * view * model * vec4(aPosition, 1.0);
}
#version 430
layout (location = 0) in vec4 aPosition;
layout (location = 1) in vec4 aNormal;
layout (location = 2) in vec4 aTexCoords;

uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;
uniform mat4 normalMat;
uniform bool showTexture;

out VS_OUT
{
  vec3 vNormal;
  vec3 vPosW;
  vec2 vTexCoords;
} vs_out;

//out vec3 WorldPos_CS_in;
//out vec2 TexCoord_CS_in;
//out vec3 Normal_CS_in;

void main()
{
	//Normal_CS_in = (normalize(normalMat * vec4(aNormal.xyz, 1.0))).xyz;
	//WorldPos_CS_in = (model * vec4(aPosition.xyz, 1.0)).xyz;
	//TexCoord_CS_in = showTexture ? TexCoord_CS_in = aTexCoords.xy : vec2(0.0);
  
	vs_out.vNormal = (normalize(normalMat * vec4(aNormal.xyz, 1.0))).xyz;
	vs_out.vPosW = (model * vec4(aPosition.xyz, 1.0)).xyz;
	vs_out.vTexCoords = showTexture ? vs_out.vTexCoords = aTexCoords.xy : vec2(0.0);

	gl_Position = proj * view * model * vec4(aPosition.xyz, 1.0);
}				
//
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
  vec2 uv0;
  vec2 uv1;
} vs_out;

void main()
{
	vs_out.vNormal = (normalize(normalMat * vec4(aNormal.xyz, 1.0))).xyz;
	vs_out.vPosW = (model * vec4(aPosition.xyz, 1.0)).xyz;
	vs_out.vTexCoords = showTexture ? vs_out.vTexCoords = aTexCoords.xy : vec2(0.0);

  vs_out.uv0 = fract( vs_out.vTexCoords.xy );
  vs_out.uv1 = fract( vs_out.vTexCoords.xy + vec2(0.5,0.5) ) - vec2(0.5,0.5);

  //int indexOffset = gl_VertexID % 3;
  //int indexV0; int indexV1; int indexV2;
  //indexV0 = gl_VertexID - indexOffset + 0; 
  //indexV1 = gl_VertexID - indexOffset + 1; 
  //indexV2 = gl_VertexID - indexOffset + 2;  

  //vs_out.vertexID = gl_VertexID;

  //vs_out.triangleIndexes[0] = indexV0;
  //vs_out.triangleIndexes[1] = indexV1;
  //vs_out.triangleIndexes[2] = indexV2;

	gl_Position = proj * view * model * vec4(aPosition.xyz, 1.0);
}				
//
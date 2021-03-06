﻿#version 430
layout (location = 0) in vec4 aPosition;
layout (location = 1) in vec4 aNormal;
layout (location = 2) in vec4 aTexCoords;
uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;
uniform mat4 normalMat;
uniform bool showTexture;
out vec3 vNormal;
out vec3 vPosW;
out vec2 vTexCoords;
//out vec2 uv0;
//out vec2 uv1;

void main()
{
	vNormal = (normalize(normalMat * vec4(aNormal.xyz, 1.0))).xyz;
	vPosW = (model * vec4(aPosition.xyz, 1.0)).xyz;
	vTexCoords = showTexture ? vTexCoords = aTexCoords.xy : vec2(0.0);
  
  //float delta = 0.01f;
  //uv0 = fract(aTexCoords.xy);
  //uv1 = fract(aTexCoords.xy + vec2(delta, delta)) - vec2(delta, delta);

	gl_Position = proj * view * model * vec4(aPosition.xyz, 1.0);
}				
//
#version 430

layout (location = 0) in vec4 aPosition;
layout (location = 1) in vec4 aNormal;
layout (location = 2) in vec4 aTexCoords;

uniform mat4 proj;
uniform mat4 view;
uniform mat4 normalMat;
uniform mat4 objectTransform;

//out vec4 vNormal;
//out vec3 vPosition;
out vec4 vTexCoord;
//out vec4 vModelView;

void main()
{
	//vec3 n = normalize((normalMat*aNormal).xyz);
	//vec3 p = (objectTransform*vec4(aPosition.xyz, 1.0)).xyz;

	//vNormal.xyz = n;
	//vPosition.xyz = p;
	vTexCoord = aTexCoords;
	//vModelView = view * objectTransform * vec4(aPosition.xyz, 1.0);
	// calculate window-space point size
	gl_Position = proj * view * objectTransform * aPosition;
}
//
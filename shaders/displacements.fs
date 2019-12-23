#version 430

//in vec4 vNormal;
//in vec4 vLight;
//in mat4 vModelViewLight;
//in vec3 vPosition;
//in vec4 vTexCoord;
//in vec4 vModelView;

//uniform vec3 lightDir;
//uniform vec3 lightPos;
//uniform sampler2D tex;
out vec4 outFragColor;

void main()
{

	//vec3 lVec = normalize(vPosition.xyz - (lightPos));
	//vec3 lPos = vec3(vLight.xyz / vLight.w);
	//vec3 n = vNormal.xyz;
	vec3 diffuse = vec3(1.0, 0.0, 0.0);//texture2D(tex, vTexCoord.xy).xyz;

	outFragColor = vec4(diffuse, 1.0);
}
//
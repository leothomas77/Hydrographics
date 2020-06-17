#version 430

uniform vec3 uLPos;
uniform vec4 uLColor;
uniform vec4 uColor;
uniform vec3 uCamPos;
uniform vec4 uAmbient;
uniform vec4 uSpecular;
uniform uint uSpecularExpoent;
uniform bool showTexture;
uniform sampler2D tex;
//inputs from geometry shader
in GS_OUT
{
    vec3 vNormal;
    vec3 vPosW;
    vec2 vTexCoords;
} fs_in;

out vec4 outFragColor;

void main(void) {
	vec4 diffuse  = vec4(0.0);
	vec4 specular = vec4(0.0);
  vec4 textureBorderColor = vec4(0.0, 0.0, 0.0, 0.0);
  // always show front facing (avoid black color showing)
	vec3 normal  = gl_FrontFacing ? normalize(-fs_in.vNormal) : normalize(fs_in.vNormal);
	vec4 lColor	 = uLColor;
	// material properties
	vec4 matAmb	 = uAmbient;

  vec4 matDif;
	matDif  = showTexture ? texture(tex, fs_in.vTexCoords) : uColor;

	vec4 matSpec = uSpecular;
	//ambient
	vec4 ambient = matAmb;
	//diffuse
	vec3 pos = normalize(fs_in.vPosW);
	vec3 vL = normalize(uLPos - pos);
	float NdotL = dot(normal, vL);
	if (NdotL > 0)
	{
	  diffuse = matDif * NdotL;
	}
	//specular
	vec3 vV = normalize(uCamPos - pos);
	vec3 vR = normalize(reflect(-vL, normal));
	float RdotV = dot(vV, vR);
	if (RdotV > 0)
	{
	  specular = matSpec * pow(RdotV, uSpecularExpoent);
	}
	// should use other components to apply phong
  outFragColor = matDif;
}
//
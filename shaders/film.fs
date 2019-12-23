#version 430
in vec3 vNormal;
in vec3 vPosW;
in vec2 vTexCoords;
uniform vec3 uLPos;
uniform vec4 uLColor;
uniform vec4 uColor;
uniform vec3 uCamPos;
uniform vec4 uAmbient;
uniform vec4 uSpecular;
uniform uint uSpecularExpoent;
uniform bool showTexture;
uniform sampler2D tex;
out vec4 outFragColor;
void main(void) {
	vec4 diffuse  = vec4(0.0);
	vec4 specular = vec4(0.0); 

	vec3 normal  = normalize(vNormal);
	vec4 lColor	 = uLColor;
	// material properties
	vec4 matAmb	 = uAmbient;
	vec4 matDif  = showTexture ? vec4(texture(tex, vTexCoords).rgb, 1.0) : uColor;
	vec4 matSpec = uSpecular;
	//ambient
	vec4 ambient = matAmb;
	//diffuse
	vec3 pos = normalize(vPosW);
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
	outFragColor = clamp((ambient + diffuse + specular) * lColor, 0.0, 1.0);
}
//
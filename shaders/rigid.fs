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
	vec4 lColor		= uLColor; 
	//material properties
	vec4 matAmb		= uAmbient;
	vec4 matDif		= showTexture ? texture(tex, vTexCoords) : uColor;
	vec4 matSpec	= uSpecular;
	vec4 ambient	= matAmb;
	vec3 vL = normalize(uLPos - vPosW);
	vec3 normal = normalize(vNormal);
	float NdotL = dot(vL, normal);
	//diffuse	
	float cTheta = max(NdotL, 0.0); 
	vec4 diffuse = matDif * cTheta; 
	//specular
	vec3 vV = normalize(uCamPos - vPosW); 
	vec3 vR = normalize(reflect(-vL, normal)); 
	float cOmega = max(dot(vV, vR), 0.0); 
	vec4 specular =	matSpec * pow(cOmega, uSpecularExpoent);
	//trick to avoid hilights when angle greater than 90
	if (NdotL < 0) 
	{
		specular = vec4(0);
	}
	outFragColor = clamp((ambient + diffuse + specular) * lColor, 0.0, 1.0);
} 	
//
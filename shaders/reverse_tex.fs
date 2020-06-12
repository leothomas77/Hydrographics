#version 430
//in vec3 vNormal;
//in vec3 vPosW;
//in vec2 vTexCoords;

//in vec3 WorldPos_FS_in;
//in vec2 TexCoord_FS_in;
//in vec3 Normal_FS_in;


//in vec3 WorldPos_GS_in;
//in vec2 TexCoord_GS_in;
//in vec3 Normal_GS_in;


//in vec2 uv0;
//in vec2 uv1;

uniform vec3 uLPos;
uniform vec4 uLColor;
uniform vec4 uColor;
uniform vec3 uCamPos;
uniform vec4 uAmbient;
uniform vec4 uSpecular;
uniform uint uSpecularExpoent;
uniform bool showTexture;
uniform sampler2D tex;

in GS_OUT
{
    vec3 vNormal;
    vec3 vPosW;
    vec2 vTexCoords;
} gs_out;

out vec4 outFragColor;

void main(void) {
	vec4 diffuse  = vec4(0.0);
	vec4 specular = vec4(0.0);
  //vec4 textureBorderColor = vec4(0.0, 0.0, 0.0, 0.0);
  vec4 textureBorderColor = vec4(0.0, 0.0, 0.0, 0.0);
  // always show front facing (avoid black color showing)
	vec3 normal  = gl_FrontFacing ? normalize(-gs_out.vNormal) : normalize(gs_out.vNormal);
	vec4 lColor	 = uLColor;
	// material properties
	vec4 matAmb	 = uAmbient;

  vec4 matDif;
	//vec2 seamlessCoords;

	//seamlessCoords.x = ( fwidth( uv0.x ) < fwidth( uv1.x )-0.001 )? uv0.x : uv1.x ;
	//seamlessCoords.y = ( fwidth( uv0.y ) < fwidth( uv1.y )-0.001 )? uv0.y : uv1.y ;

	//vec4 colorRedGreen = vec4(1.0, 1.0, 0.0, 1.0);
	
	//colorRedGreen.r *= vTexCoords.x;
	//colorRedGreen.g *= vTexCoords.y;
	matDif  = showTexture ? texture(tex, gs_out.vTexCoords) : uColor;

	vec4 matSpec = uSpecular;
	//ambient
	vec4 ambient = matAmb;
	//diffuse
	vec3 pos = normalize(gs_out.vPosW);
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
	//outFragColor = clamp((ambient + diffuse + specular) * lColor, 0.0, 1.0);
	outFragColor = matDif;
}
//
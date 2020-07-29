#version 430

uniform vec3 uLPos;
uniform vec4 uLColor;
uniform vec4 uColor;
uniform vec3 uCamPos;
uniform vec4 uAmbient;
uniform vec4 uSpecular;
uniform uint uSpecularExpoent;
uniform bool uColorCompensation;
uniform bool showTexture;
uniform sampler2D tex;
//inputs from geometry shader
in GS_OUT
{
    vec3 vNormal;
    vec3 vPosW;
    vec2 vTexCoords;
    vec4 vColor;
		vec2 uv0;
		vec2 uv1;

} fs_in;

out vec4 outFragColor;

void main(void) {
	vec4 diffuse  = vec4(0.0);
	vec4 specular = vec4(0.0);
	vec3 normal  = normalize(-fs_in.vNormal);
	vec4 lColor	 = uLColor;
	// material properties
	vec4 matAmb	 = uAmbient;

  vec4 matDif;
  vec4 bgColor = vec4(1.0f);
  //vec2 uvT; //Tarini 
  //uvT.x = ( fwidth( fs_in.uv0.x ) < fwidth( fs_in.uv1.x )-0.001 )? fs_in.uv0.x : fs_in.uv1.x;
	//uvT.y = ( fwidth( fs_in.uv0.y ) < fwidth( fs_in.uv1.y )-0.001 )? fs_in.uv0.y : fs_in.uv1.y;

  //uvT.x = ( fwidth( fs_in.uv0.x ) < fwidth( fs_in.uv0.x +0.001) )? fs_in.uv0.x : fs_in.uv0.x +000.1;
	//uvT.y = ( fwidth( fs_in.uv0.y ) < fwidth( fs_in.uv0.y +0.001) )? fs_in.uv0.y : fs_in.uv0.y +000.1;

  matDif  = showTexture ? texture(tex, fs_in.vTexCoords) : fs_in.vColor;
  if (fs_in.vTexCoords.x >= 0.0f && fs_in.vTexCoords.y >= 0.0f)
  {
    bgColor = fs_in.vColor;
  }
  //matDif  = showTexture ? texture(tex, uvT) : uColor;


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

  if (uColorCompensation)
  {
    matDif = matDif * bgColor;  
  }
  outFragColor = matDif;
}
//
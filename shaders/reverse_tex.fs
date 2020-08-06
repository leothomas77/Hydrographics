#version 430

uniform vec3 uLPos;
//uniform vec4 uLColor;
uniform vec4 uColor;
uniform vec3 uCamPos;
//uniform vec4 uAmbient;
//uniform vec4 uSpecular;
//uniform uint uSpecularExpoent;
//uniform bool showTexture;
uniform sampler2D reverseTexture;
uniform sampler1D colorMap;
uniform int textureMode;

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
	vec3 normal  = normalize(-fs_in.vNormal);
  vec4 matDif = vec4(1.0f);

  float compensationFactor = 1.0f - fs_in.vColor.x;


  if (fs_in.vTexCoords.x >= 0.0f && fs_in.vTexCoords.y >= 0.0f)
  {
    matDif  = texture(reverseTexture, fs_in.vTexCoords);

    if (textureMode == 2)
    {
      matDif *=  compensationFactor;
    }
  }
  
  if (textureMode == 3)
  {
    matDif = vec4(compensationFactor);
  }

  if (textureMode == 4)
  {
    matDif = texture(colorMap, fs_in.vColor.x);
  }

  outFragColor = matDif;
}
//
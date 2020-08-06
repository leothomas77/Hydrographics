#version 430 core
  layout(triangles) in; 
  layout(triangle_strip, max_vertices = 36) out; 
  uniform float uMaxDistanceUV; 
  uniform float uNearDistanceUV; 
  uniform float uWeight1; 
  uniform float uWeight2; 
  uniform sampler2D tex; 
  uniform sampler2D heatmap; 
  
in VS_OUT
{ 
vec3 vNormal; 
vec3 vPosW; 
vec2 vTexCoords; 
} gs_in[]; 

out GS_OUT
{ 
vec3 vNormal; 
vec3 vPosW; 
vec2 vTexCoords; 
} gs_out; 


void EmitTextureVertex(vec4 pos, vec3 posW, vec2 texCoords)
{
  gl_Position = pos;
  gs_out.vNormal = gs_in[0].vNormal;
  gs_out.vTexCoords = texCoords;
  gs_out.vPosW = posW;
  EmitVertex();
}

void ComputeTexDistance(vec2 texV0, vec2 texV1, vec2 texV2, inout float distance01, inout float distance02, inout float distance12)
{
  distance01 = length(texV0 - texV1);
  distance02 = length(texV0 - texV2);
  distance12 = length(texV1 - texV2);
}

void ComputeCrossSeam(float maxDistanceUV, float distance01, float distance02, float distance12, inout bool cross01, inout bool cross02, inout bool cross12)
{
  cross01 = distance01 > maxDistanceUV;
  cross02 = distance02 > maxDistanceUV;
  cross12 = distance12 > maxDistanceUV;
}

bool isSeam(float maxDistanceUV, bool cross01, bool cross02, bool cross12, bool crossAll)
{
  return (cross01 || cross02 || cross12 || crossAll); 
}

bool v0_CrossTheSeam(bool cross01, bool cross02)
{
  return (cross01 && cross02);
}

bool v1_CrossTheSeam(bool cross01, bool cross12)
{
  return (cross01 && cross12);
}

bool v2_CrossTheSeam(bool cross02, bool cross12)
{
  return (cross02 && cross12);
}

bool all_CrossTheSeam(float maxDistanceUV, float distance01, float distance02, float distance12)
{
  float crossAllMaxDistanceUV = 0.33f * maxDistanceUV;
  return (distance01 > crossAllMaxDistanceUV && distance02 > crossAllMaxDistanceUV && distance12 > crossAllMaxDistanceUV);
}

void BuildCrossAllStrips()
{

}

void EmitStrips(vec4 far, vec3 farW, vec2 farTexCoords, vec4 near1, vec3 nearW1, vec2 nearTexCoords1, vec4 near2, vec3 nearW2, vec2 nearTexCoords2)
{
//parei aqui - indo para o tesselation shader
}

void EmitStrips(int farVertexIndex, int nearVertex1, int nearVertex2, float w1, float w2)
{
      // v0 cross the seam
      // strip 1
      EmitTextureVertex(nearVertex1);
      EmitWeightedTexVertex(farVertexIndex, nearVertex1, w1, w2);      
      EmitWeightedTexVertex(nearVertex1, nearVertex2, 0.5f, 0.5f);      
      EmitWeightedTexVertex(farVertexIndex, nearVertex2, w1, w2);      
      EmitTextureVertex(nearVertex2);

      EndPrimitive();
      
      // strip 2
      EmitWeightedTexVertex(nearVertex1, farVertexIndex, w1, w2);
      EmitTextureVertex(farVertexIndex);
      EmitWeightedTexVertex(nearVertex2, farVertexIndex, w1, w2);      

      EndPrimitive();

}


void BuildSubtriangles(vec4 pos0, vec4 pos1, vec4 pos2, vec3 posW0, vec3 posW1, vec3 posW2, 
  vec2 texCoords0, vec2 texCoords1, vec2 texCoords2, 
  float maxDistanceUV, int divisions, bool cross01, bool cross02, bool cross12, bool crossAll)
{
  vec4 far; vec4 near1; vec4 near2;  
  vec3 farW; vec3 nearW1; vec3 nearW2; 
  vec2 farTexCoords; vec2 nearTexCoords1; vec2 nearTexCoords2;

  if (crossAll)
  {
    BuildCrossAllStrips();

    return;

  } else if (cross01) {
    far = pos0;                near1 = pos1;                near2 = pos2;
    farW = posW0;              nearW1 = posW1;              nearW2 = posW2;
    farTexCoords = texCoords0; nearTexCoords1 = texCoords1; nearTexCoords2 = texCoords2;
      
    EmitStrips(far, farW, farTexCoords, near1, nearW1, nearTexCoords1, near2, nearW2, nearTexCoords2);

  } else if (cross02) {
    far = pos1;                near1 = pos2;                near2 = pos0;
    farW = posW1;              nearW1 = posW2;              nearW2 = posW0;
    farTexCoords = texCoords1; nearTexCoords1 = texCoords2; nearTexCoords2 = texCoords0;
  
    EmitStrips(far, farW, farTexCoords, near1, nearW1, nearTexCoords1, near2, nearW2, nearTexCoords2);

  } else if (cross12) {
    far = pos2;                near1 = pos0;                near2 = pos1;
    farW = posW2;              nearW1 = posW0;              nearW2 = posW1;
    farTexCoords = texCoords2; nearTexCoords1 = texCoords0; nearTexCoords2 = texCoords1;  

    EmitStrips(far, farW, farTexCoords, near1, nearW1, nearTexCoords1, near2, nearW2, nearTexCoords2);

  }

}

void main(void) {

  float maxDistanceUV = uMaxDistanceUV;
  int subdivisions = 2;
  
  vec2 texCoords0 = gs_in[0].vTexCoords; vec2 texCoords1 = gs_in[1].vTexCoords; vec2 texCoords2 = gs_in[2].vTexCoords;
  vec3 posW0 = gs_in[0].vPosW; vec3 posW1 = gs_in[1].vPosW; vec3 posW2 = gs_in[2].vPosW;  
  vec4 pos0 = gl_in[0].gl_Position; vec4 pos1 = gl_in[1].gl_Position; vec4 pos2 = gl_in[2].gl_Position; 

  float texDistance01; float texDistance02; float texDistance12;
  bool cross01; bool cross02; bool cross12;

  ComputeTexDistance(texCoords0, texCoords1, texCoords2, texDistance01, texDistance02, texDistance12);
  ComputeCrossSeam(maxDistanceUV, texDistance01, texDistance02, texDistance12, cross01, cross02, cross12);

  bool crossAll = all_CrossTheSeam(maxDistanceUV, texDistance01, texDistance02, texDistance12);
  if (isSeam(maxDistanceUV, cross01, cross02, cross12, crossAll))
  {
    BuildSubtriangles(pos0, pos1, pos2, posW0, posW1, posW2, texCoords0, texCoords1, texCoords2, maxDistanceUV, subdivisions, cross01, cross02, cross12, crossAll);
  }
  else
  {
    // pass-through original vertices
    EmitTextureVertex(pos0, posW0, texCoords0);
    EmitTextureVertex(pos1, posW1, texCoords1);
    EmitTextureVertex(pos2, posW2, texCoords2);

    EndPrimitive();

  }

}


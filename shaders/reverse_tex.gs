#version 430 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 36) out;

uniform float uMaxDistanceUV;

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

void EmitTextureVertex(int index)
{
  gl_Position = gl_in[index].gl_Position;
  gs_out.vNormal = gs_in[index].vNormal;
  gs_out.vTexCoords = gs_in[index].vTexCoords;
  gs_out.vPosW = gs_in[index].vPosW;
  EmitVertex();
}

void EmitWeightedTexVertex(int index1, int index2, float weight1, float weight2)
{
  gl_Position = 0.5f * (gl_in[index1].gl_Position + gl_in[index2].gl_Position);
  gs_out.vNormal = gs_in[index1].vNormal; // the mesh is always a plane assumes always the same normal
  gs_out.vTexCoords = (weight1 * gs_in[index1].vTexCoords + weight2 * gs_in[index2].vTexCoords); // vertex with greater texture weight
  gs_out.vPosW = 0.5 * (gs_in[index1].vPosW + gs_in[index2].vPosW);
  EmitVertex();
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

void ComputeHalfTextures(vec2 texV0, vec2 texV1, vec2 texV2, inout vec2 halfTex01, inout vec2 halfTex02, inout vec2 halfTex12)
{
  halfTex01 = 0.5f * (texV0 + texV1);
  halfTex02 = 0.5f * (texV0 + texV2);
  halfTex12 = 0.5f * (texV1 + texV2);
}

void ComputeHalfDistances(vec4 v0, vec4 v1, vec4 v2, inout vec4 half01, inout vec4 half02, inout vec4 half12)
{
  half01 = 0.5f * (v0 + v1);
  half02 = 0.5f * (v0 + v2);
  half12 = 0.5f * (v1 + v2);
}


void main(void) {

  float maxDistanceUV = uMaxDistanceUV;
  int edgeSubdivisions = 2;

  float texDistance01, texDistance02, texDistance12;
  bool cross01, cross02, cross12;

  vec2 halfTex01, halfTex02, halfTex12;
  vec4 half01, half02, half12;
  

  ComputeHalfTextures(gs_in[0].vTexCoords, gs_in[1].vTexCoords, gs_in[2].vTexCoords, halfTex01, halfTex02, halfTex12);
  ComputeHalfDistances(gl_in[0].gl_Position, gl_in[1].gl_Position, gl_in[2].gl_Position, half01, half02, half12);

  ComputeTexDistance(gs_in[0].vTexCoords, gs_in[1].vTexCoords, gs_in[2].vTexCoords, texDistance01,texDistance02, texDistance12);

  ComputeCrossSeam(maxDistanceUV, texDistance01, texDistance02, texDistance12, cross01, cross02, cross12);

  bool crossAll = all_CrossTheSeam(maxDistanceUV, texDistance01, texDistance02, texDistance12);
  if (isSeam(maxDistanceUV, cross01, cross02, cross12, crossAll))
  {
    if (crossAll)
    {
      // all cross the stream
      // strip 1
      EmitTextureVertex(2);
      EmitWeightedTexVertex(0, 2, 0.5f, 0.5f);      
      EmitWeightedTexVertex(1, 2, 0.5f, 0.5f);      

      EndPrimitive();
   
      // strip 2
      EmitWeightedTexVertex(1, 2, 0.5f, 0.5f);      
      EmitWeightedTexVertex(0, 1, 0.5f, 0.5f);      
      EmitTextureVertex(1);

      EndPrimitive();
      
      // strip 3
      EmitWeightedTexVertex(0, 2, 0.5f, 0.5f);      
      EmitTextureVertex(0);
      EmitWeightedTexVertex(0, 1, 0.5f, 0.5f);      

      EndPrimitive();

      // strip 4 center  
      EmitWeightedTexVertex(1, 2, 0.5f, 0.5f);      
      EmitWeightedTexVertex(0, 2, 0.5f, 0.5f);      
      EmitWeightedTexVertex(0, 1, 0.5f, 0.5f);      
      
      EndPrimitive();

    } 
    else if (v0_CrossTheSeam(cross01, cross02))
    {
      // v0 cross the seam
      // strip 1
      EmitTextureVertex(1);
      EmitWeightedTexVertex(0, 1, 0.0f, 1.0f);      
      EmitWeightedTexVertex(1, 2, 0.5f, 0.5f);      
      EmitWeightedTexVertex(0, 2, 0.0f, 1.0f);      
      EmitTextureVertex(2);

      EndPrimitive();
      
      // strip 2
      EmitWeightedTexVertex(0, 1, 1.0f, .0f);
      EmitTextureVertex(0);
      EmitWeightedTexVertex(0, 2, 1.0f, .0f);      

      EndPrimitive();

    } 
    else if (v1_CrossTheSeam(cross01, cross12))
    {
      //v1 cross the seam
      // strip 1
      EmitTextureVertex(2);
      EmitWeightedTexVertex(1, 2, 0.0f, 1.0f);      
      EmitWeightedTexVertex(0, 2, 0.5f, 0.5f);      
      EmitWeightedTexVertex(0, 1, 1.0f, 0.0f);      
      EmitTextureVertex(0);

      EndPrimitive();
      
      // strip 2
      EmitWeightedTexVertex(1, 2, 1.0f, 0.0f);
      EmitTextureVertex(1);
      EmitWeightedTexVertex(0, 1, 0.0f, 1.0f);      

      EndPrimitive();

    } 
    else if (v2_CrossTheSeam(cross02, cross12))
    {
      //v2 cross the seam
      // strip 1
      EmitTextureVertex(0);
      EmitWeightedTexVertex(0, 2, 1.0f, 0.0f);      
      EmitWeightedTexVertex(0, 1, 0.5f, 0.5f);      
      EmitWeightedTexVertex(1, 2, 1.0f, 0.0f);      
      EmitTextureVertex(1);

      EndPrimitive();
      
      // strip 2
      EmitWeightedTexVertex(1, 2, 0.0f, 1.0f);
      EmitTextureVertex(2);
      EmitWeightedTexVertex(0, 1, 0.0f, 1.0f);      

      EndPrimitive();

    }    

  }
  else 
  {
    // pass-through original vertices
    EmitTextureVertex(0);
    EmitTextureVertex(1);
    EmitTextureVertex(2);

    EndPrimitive();

  }

 
}
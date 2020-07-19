#version 430 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 36) out;
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
  vec2 uv0;
  vec2 uv1;
} gs_in[];

out GS_OUT
{
    vec3 vNormal;
    vec3 vPosW;
    vec2 vTexCoords;
		vec2 uv0;
		vec2 uv1;
} gs_out;

void EmitTextureVertex(int index)
{
  gl_Position = gl_in[index].gl_Position;
  gs_out.vNormal = gs_in[index].vNormal;
  gs_out.vTexCoords = gs_in[index].vTexCoords;
  gs_out.uv0 = gs_in[index].uv0;
  gs_out.uv1 = gs_in[index].uv1;
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

void EmitDirectionTexVertex(int indexFrom, int indexTo, vec2 texCoordFrom, vec2 texCoordTo, float epsilon)
{
  gl_Position = 0.5f * (gl_in[indexFrom].gl_Position + gl_in[indexTo].gl_Position);
  gs_out.vNormal = gs_in[indexFrom].vNormal; // the mesh is always a plane assumes always the same normal
  
  vec2 textDirection = normalize(texCoordTo - texCoordFrom);
  
  gs_out.vTexCoords =  texCoordFrom + textDirection * epsilon;
  gs_out.vPosW = 0.5 * (gs_in[indexFrom].vPosW + gs_in[indexTo].vPosW);
  EmitVertex();
}

void EmitStrips(int farVertexIndex, int nearVertex1, int nearVertex2, float w1, float w2)
{
      // v0 cross the seam
      // strip 1
      EmitTextureVertex(nearVertex1);
      EmitWeightedTexVertex(farVertexIndex, nearVertex1, .01f, .99f);
      EmitWeightedTexVertex(nearVertex1, nearVertex2, .5f, .5f);      
      EmitWeightedTexVertex(farVertexIndex, nearVertex2, .01f, .99f);   
      EmitTextureVertex(nearVertex2);
      EndPrimitive();
      
      // strip 2
      EmitWeightedTexVertex(nearVertex1, farVertexIndex, .01f, .99f);
      EmitTextureVertex(farVertexIndex);
      EmitWeightedTexVertex(nearVertex2, farVertexIndex, .01f, .99f);      
      EndPrimitive();
}

void EmitStrips(int farVertexIndex, int nearVertex1, int nearVertex2, float epsilon)
{
      vec2 texFar = gs_in[farVertexIndex].vTexCoords;
      vec2 texNear1 = gs_in[nearVertex1].vTexCoords;
      vec2 texNear2 = gs_in[nearVertex2].vTexCoords;

      // v0 cross the seam
      // strip 1
      EmitTextureVertex(nearVertex1);
      EmitDirectionTexVertex(farVertexIndex, nearVertex1, texNear1, texFar, epsilon);
      EmitWeightedTexVertex(nearVertex1, nearVertex2, .5f, .5f);      
      EmitDirectionTexVertex(farVertexIndex, nearVertex2, texNear2, texFar, epsilon);
      EmitTextureVertex(nearVertex2);
      EndPrimitive();
            
      // strip 2
      EmitDirectionTexVertex(nearVertex1, farVertexIndex, texFar, texNear1, epsilon);
      EmitTextureVertex(farVertexIndex);
      EmitDirectionTexVertex(nearVertex2, farVertexIndex, texFar, texNear2, epsilon);
      EndPrimitive();
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

/*

Seam fix scheme
  near1    near2 (near vertex in the texture)
      _ * _  
    *       * 
    | / |  /  
  _ * _ * _ _ _ seam  
    |  /
    *
   far (vertex far then the 2 others)
    
*/

void main(void) {

  float maxDistanceUV = uMaxDistanceUV;
  float epsilon = uNearDistanceUV;
  float w1 = uWeight1; 
  float w2 = uWeight2;

  float w3 = uWeight1;
  float w4 = uWeight2;

  float texDistance01, texDistance02, texDistance12;
  bool cross01, cross02, cross12;
  vec2 halfTex01, halfTex02, halfTex12;

  ComputeHalfTextures(gs_in[0].vTexCoords, gs_in[1].vTexCoords, gs_in[2].vTexCoords, halfTex01, halfTex02, halfTex12);
  ComputeTexDistance(gs_in[0].vTexCoords, gs_in[1].vTexCoords, gs_in[2].vTexCoords, texDistance01, texDistance02, texDistance12);
  ComputeCrossSeam(maxDistanceUV, texDistance01, texDistance02, texDistance12, cross01, cross02, cross12);

  bool crossAll = all_CrossTheSeam(maxDistanceUV, texDistance01, texDistance02, texDistance12);
  
  if (isSeam(maxDistanceUV, cross01, cross02, cross12, crossAll))
  {
    if (crossAll)
    {
      // all cross the stream
      // strip 1
      EmitTextureVertex(1);
      EmitDirectionTexVertex(1, 0, gs_in[1].vTexCoords, gs_in[0].vTexCoords, epsilon);
      EmitDirectionTexVertex(1, 2, gs_in[1].vTexCoords, gs_in[2].vTexCoords, epsilon);
      EndPrimitive();
   
      // strip 2
      EmitDirectionTexVertex(2, 1, gs_in[2].vTexCoords, gs_in[1].vTexCoords, epsilon);
      EmitDirectionTexVertex(2, 0, gs_in[2].vTexCoords, gs_in[0].vTexCoords, epsilon);
      EmitTextureVertex(2);
      EndPrimitive();
      
      // strip 3
      EmitDirectionTexVertex(0, 1, gs_in[0].vTexCoords, gs_in[1].vTexCoords, epsilon);
      EmitTextureVertex(0);
      EmitDirectionTexVertex(0, 2, gs_in[0].vTexCoords, gs_in[2].vTexCoords, epsilon);
      EndPrimitive();

      // strip 4 center  
      vec2 texCenter = 0.33f * (gs_in[0].vTexCoords + gs_in[1].vTexCoords + gs_in[2].vTexCoords); 
      EmitDirectionTexVertex(0, 1, texCenter, halfTex01, epsilon);
      EmitDirectionTexVertex(1, 2, texCenter, halfTex12, epsilon);
      EmitDirectionTexVertex(0, 2, texCenter, halfTex02, epsilon);
      EndPrimitive();

    } 
    else if (v0_CrossTheSeam(cross01, cross02))
    {

      int farVertexIndex = 0;
      int nearVertex1 = 1;
      int nearVertex2 = 2; 

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, epsilon);

    } 
    else if (v1_CrossTheSeam(cross01, cross12))
    {
      
      int farVertexIndex = 1;
      int nearVertex1 = 2;
      int nearVertex2 = 0; 

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, epsilon);

    } 
    else if (v2_CrossTheSeam(cross02, cross12))
    {

      int farVertexIndex = 2;
      int nearVertex1 = 0;
      int nearVertex2 = 1; 

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, epsilon);

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
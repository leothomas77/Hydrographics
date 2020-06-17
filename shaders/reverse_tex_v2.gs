#version 430 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 36) out;
uniform float uMaxDistanceUV;
uniform float uNearDistanceUV;
uniform float uWeight1;
uniform float uWeight2;
uniform sampler2D tex;
uniform sampler2D heatmap;

in TES_OUT
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

void EmitWeightedTexVertexCenter(int index1, int index2, vec2 texCoord1, vec2 texCoord2, vec2 texCoord3, float epsilon)
{
  gl_Position = 0.5f * (gl_in[index1].gl_Position + gl_in[index2].gl_Position);
  gs_out.vNormal = gs_in[index1].vNormal; // the mesh is always a plane assumes always the same normal
  
  vec2 centerTexCoord = 0.33f * (texCoord1 + texCoord2 + texCoord3);
  vec2 centerTexDirection = normalize(centerTexCoord - texCoord1);

  gs_out.vTexCoords =  texCoord1 + centerTexDirection * epsilon;
  gs_out.vPosW = 0.5 * (gs_in[index1].vPosW + gs_in[index2].vPosW);
  EmitVertex();
}

void EmitTexVertexByNearColor(int index1, int index2, vec2 texCoords)
{
  gl_Position = 0.5f * (gl_in[index1].gl_Position + gl_in[index2].gl_Position);
  gs_out.vNormal = gs_in[index1].vNormal; // the mesh is always a plane assumes always the same normal
  gs_out.vTexCoords = texCoords;
  gs_out.vPosW = 0.5 * (gs_in[index1].vPosW + gs_in[index2].vPosW);
  EmitVertex();
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

vec2 GetTexCoordsByNearColor(vec2 texCoords, float epsilon)
{
  vec4 texColor  = texture(tex, texCoords);

  float distances[8];
  vec2 nearTexCoords[8];

  distances[0] = length(texColor - texture(tex, texCoords + vec2(0.0f, epsilon)));
  distances[1] = length(texColor - texture(tex, texCoords + vec2(0.0f, -epsilon)));
  distances[2] = length(texColor - texture(tex, texCoords + vec2(-epsilon, 0.0f)));
  distances[3] = length(texColor - texture(tex, texCoords + vec2(epsilon, 0.0f)));
  distances[4] = length(texColor - texture(tex, texCoords + vec2(epsilon, epsilon)));
  distances[5] = length(texColor - texture(tex, texCoords + vec2(epsilon, -epsilon)));
  distances[6] = length(texColor - texture(tex, texCoords + vec2(-epsilon, epsilon)));
  distances[7] = length(texColor - texture(tex, texCoords + vec2(-epsilon, -epsilon)));


  nearTexCoords[0] = texCoords + vec2(0.0f, epsilon);
  nearTexCoords[1] = texCoords + vec2(0.0f, -epsilon);
  nearTexCoords[2] = texCoords + vec2(-epsilon, 0.0f);
  nearTexCoords[3] = texCoords + vec2(epsilon, 0.0f);
  nearTexCoords[4] = texCoords + vec2(epsilon, epsilon);
  nearTexCoords[5] = texCoords + vec2(epsilon, -epsilon);
  nearTexCoords[6] = texCoords + vec2(-epsilon, epsilon);
  nearTexCoords[7] = texCoords + vec2(-epsilon, -epsilon);

  int nearColorIndex = 0;
  for (int i; i < 8; i++)
  {
    if (distances[i] < distances[nearColorIndex])
    {
      nearColorIndex = i;
    }
  }

  return nearTexCoords[nearColorIndex];

}

void main(void) {

  float maxDistanceUV = uMaxDistanceUV;
  float epsilon = uNearDistanceUV;
  float w1 = uWeight1; 
  float w2 = uWeight2;

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
      EmitTextureVertex(1);
      EmitWeightedTexVertex(1, 0, w1, w2);      
      EmitWeightedTexVertex(1, 2, w1, w2);      

      EndPrimitive();
   
      // strip 2
      EmitWeightedTexVertex(2, 1, w1, w2);      
      EmitWeightedTexVertex(2, 0, w1, w2);      
      EmitTextureVertex(2);

      EndPrimitive();
      
      // strip 3
      EmitWeightedTexVertex(0, 1, w1, w2);      
      EmitTextureVertex(0);
      EmitWeightedTexVertex(0, 2, w1, w2);      

      EndPrimitive();

      // strip 4 center  

      EmitWeightedTexVertexCenter(0, 1, halfTex01, halfTex02, halfTex12, epsilon);
      EmitWeightedTexVertexCenter(0, 2, halfTex02, halfTex12, halfTex01, epsilon);
      EmitWeightedTexVertexCenter(1, 2, halfTex12, halfTex01, halfTex02, epsilon);

      EndPrimitive();

    } 
    else if (v0_CrossTheSeam(cross01, cross02))
    {
      int farVertexIndex = 0;
      int nearVertex1 = 1;
      int nearVertex2 = 2; 

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, w1, w2);

    } 
    else if (v1_CrossTheSeam(cross01, cross12))
    {
      
      int farVertexIndex = 1;
      int nearVertex1 = 2;
      int nearVertex2 = 0; 

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, w1, w2);
      
    } 
    else if (v2_CrossTheSeam(cross02, cross12))
    {

      int farVertexIndex = 2;
      int nearVertex1 = 0;
      int nearVertex2 = 1; 

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, w1, w2);

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
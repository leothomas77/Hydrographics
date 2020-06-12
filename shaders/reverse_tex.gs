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

void EmitV0()
{
  gl_Position = gl_in[0].gl_Position;
  gs_out.vNormal = gs_in[0].vNormal;
  gs_out.vTexCoords = gs_in[0].vTexCoords;
  gs_out.vPosW = gs_in[0].vPosW;
  EmitVertex();
}


void EmitV1()
{
  gl_Position = gl_in[1].gl_Position;
  gs_out.vNormal = gs_in[1].vNormal;
  gs_out.vTexCoords = gs_in[1].vTexCoords;
  gs_out.vPosW = gs_in[1].vPosW;
  EmitVertex();
}


void EmitV2()
{
  gl_Position = gl_in[2].gl_Position;
  gs_out.vNormal = gs_in[2].vNormal;
  gs_out.vTexCoords = gs_in[2].vTexCoords;
  gs_out.vPosW = gs_in[2].vPosW;
  EmitVertex();
}

void EmitHalf01(int weight1, int weight2)
{
  gl_Position = 0.5f * (gl_in[0].gl_Position + gl_in[1].gl_Position);
  gs_out.vNormal = gs_in[1].vNormal;
  gs_out.vTexCoords = 0.5 * (gs_in[weight1].vTexCoords + gs_in[weight2].vTexCoords); // vertex with greater texture weight
  gs_out.vPosW = 0.5 * (gs_in[0].vPosW + gs_in[1].vPosW);
  EmitVertex();
}

void EmitHalf12(int weight1, int weight2)
{
  gl_Position = 0.5f * (gl_in[1].gl_Position + gl_in[2].gl_Position);
  gs_out.vNormal = gs_in[1].vNormal;
  gs_out.vTexCoords = 0.5 * (gs_in[weight1].vTexCoords + gs_in[weight2].vTexCoords); // vertex with greater texture weight
  gs_out.vPosW = 0.5 * (gs_in[1].vPosW + gs_in[2].vPosW);;
  EmitVertex();
}

void EmitHalf02(int weight1, int weight2)
{
  gl_Position = 0.5f * (gl_in[0].gl_Position + gl_in[2].gl_Position);
  gs_out.vNormal = gs_in[1].vNormal;
  gs_out.vTexCoords = 0.5 * (gs_in[weight1].vTexCoords + gs_in[weight2].vTexCoords); // vertex with greater texture weight
  gs_out.vPosW = 0.5 * (gs_in[0].vPosW + gs_in[2].vPosW);;
  EmitVertex();
}

bool isSeam(float maxDistanceUV, bool cross01, bool cross02, bool cross12)
{
  return (cross01 || cross02 || cross12); 
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

bool all_CrossTheSeam(bool cross01, bool cross02, bool cross12)
{
  return (cross01 && cross02 && cross12);
}


void main(void) {

  float maxDistanceUV = 0.48f; //uMaxDistanceUV;

  float distance01 = length(gs_in[0].vTexCoords-gs_in[1].vTexCoords);
  float distance02 = length(gs_in[0].vTexCoords-gs_in[2].vTexCoords);
  float distance12 = length(gs_in[1].vTexCoords-gs_in[2].vTexCoords);

  bool cross01 = distance01 > maxDistanceUV;
  bool cross02 = distance02 > maxDistanceUV;
  bool cross12 = distance12 > maxDistanceUV;

  vec2 halfTex01 = 0.5f * gs_in[0].vTexCoords-gs_in[1].vTexCoords;
  vec2 halfTex02 = 0.5f * gs_in[0].vTexCoords-gs_in[2].vTexCoords;
  vec2 halfTex12 = 0.5f * gs_in[1].vTexCoords-gs_in[2].vTexCoords;

  if (isSeam(maxDistanceUV, cross01, cross02, cross12))
  {
    if (all_CrossTheSeam(cross01, cross02, cross12))
    {
      // all cross the stream
      // strip 1
      EmitV2();
      EmitHalf02(2,2);
      EmitHalf12(2,2);

      EndPrimitive();
   
      // strip 2
      EmitHalf12(1,1);
      EmitHalf01(1,1);
      EmitV1();

      EndPrimitive();
      
      // strip 3
      EmitHalf02(0,0);
      EmitV0();
      EmitHalf01(0,0);

      EndPrimitive();

      // strip 4 center  
      EmitHalf12(1,2);
      EmitHalf02(0,2);
      EmitHalf01(0,1);
      
      /*
      gl_Position = 0.5f * (gl_in[0].gl_Position + gl_in[2].gl_Position);
      gs_out.vNormal = gs_in[1].vNormal;
      gs_out.vTexCoords = 0.33f * (gs_in[0].vTexCoords + gs_in[1].vTexCoords + gs_in[2].vTexCoords) ; // vertex with greater texture weight
      gs_out.vPosW = 0.5 * (gs_in[0].vPosW + gs_in[2].vPosW);
      EmitVertex();

      gl_Position = 0.5f * (gl_in[1].gl_Position + gl_in[2].gl_Position);
      gs_out.vNormal = gs_in[1].vNormal;
      gs_out.vTexCoords = 0.33f * (gs_in[0].vTexCoords + gs_in[1].vTexCoords + gs_in[2].vTexCoords) ; // vertex with greater texture weight
      gs_out.vPosW = 0.5 * (gs_in[1].vPosW + gs_in[2].vPosW);
      EmitVertex();

      gl_Position = 0.5f * (gl_in[0].gl_Position + gl_in[1].gl_Position);
      gs_out.vNormal = gs_in[1].vNormal;
      gs_out.vTexCoords = 0.33f * (gs_in[0].vTexCoords + gs_in[1].vTexCoords + gs_in[2].vTexCoords) ; // vertex with greater texture weight
      gs_out.vPosW = 0.5 * (gs_in[0].vPosW + gs_in[1].vPosW);
      EmitVertex();
      */

      EndPrimitive();
    } 
    else if (v0_CrossTheSeam(cross01, cross02))
    {
      // v0 cross the seam
      // strip 1
      EmitV1();
      EmitHalf01(1,1);
      EmitHalf12(1,2);
      EmitHalf02(2,2);
      EmitV2();

      EndPrimitive();
      
      // strip 2
      EmitHalf01(0,0);
      EmitV0();
      EmitHalf02(0,0);

      EndPrimitive();
    
    } 
    else if (v1_CrossTheSeam(cross01, cross12))
    {
      //v1 cross the seam
      // strip 1
      EmitV2();
      EmitHalf12(2,2);
      EmitHalf02(0,2);
      EmitHalf01(0,0);
      EmitV0();

      EndPrimitive();
      
      // strip 2
      EmitHalf12(1,1);
      EmitV1();
      EmitHalf01(1,1);

      EndPrimitive();

    } 
    else if (v2_CrossTheSeam(cross02, cross12))
    {
      //v2 cross the seam
      // strip 1
      EmitV0();
      EmitHalf02(0,0);
      EmitHalf01(0,1);
      EmitHalf12(1,1);
      EmitV1();

      EndPrimitive();
      
      // strip 2
      EmitHalf02(2,2);
      EmitV2();
      EmitHalf12(2,2);

      EndPrimitive();

    } 
    

  }
  else 
  {
    // pass-through original vertices
    gl_Position = gl_in[0].gl_Position;
    gs_out.vNormal = gs_in[0].vNormal;
    gs_out.vTexCoords = gs_in[0].vTexCoords;
    gs_out.vPosW = gs_in[0].vPosW;
    EmitVertex();

    gl_Position = gl_in[1].gl_Position;
    gs_out.vNormal = gs_in[1].vNormal;
    gs_out.vTexCoords = gs_in[1].vTexCoords;
    gs_out.vPosW = gs_in[1].vPosW;
    EmitVertex();

    gl_Position = gl_in[2].gl_Position;
    gs_out.vNormal = gs_in[2].vNormal;
    gs_out.vTexCoords = gs_in[2].vTexCoords;
    gs_out.vPosW = gs_in[2].vPosW;
    EmitVertex();

    EndPrimitive();

  }

 
}
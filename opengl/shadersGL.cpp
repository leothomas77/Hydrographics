// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 20132017 NVIDIA Corporation. All rights reserved.
#include "../source/shaders.h"
#include "../source/ColorGradient.h"

#include "../core/mesh.h"
#include "../core/tga.h"	
#include "../core/platform.h"
#include "../core/extrude.h"

#include "../external/SDL2-2.0.4/include/SDL.h"
#include "../external/glad/src/glad.c"
#include <glm/gtc/matrix_transform.hpp> 
#include "shader.h"


#define STB_IMAGE_STATIC
#ifndef STB_IMAGE_IMPLEMENTATION
  #define STB_IMAGE_IMPLEMENTATION
  #include "../source/stb_image.h"
#endif // !STB_IMAGE_IMPLEMENTATION
#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
  #define STB_IMAGE_WRITE_IMPLEMENTATION
  #include "../source/stb_image_write.h"
#endif // !STB_IMAGE_WRITE_IMPLEMENTATION

#include "imguiRenderGL.h"
#include "utilsGL.h"

GLuint g_chessboard_texture_id;
GLuint g_rigid_model_texture_id;
GLuint g_heatmap_texture_id;

GLuint s_reverseTexProgram = GLuint(-1);
GLuint s_rigidBodyProgram = GLuint(-1);
GLuint s_filmProgram = GLuint(-1);

bool g_enable_paches = false;

#ifdef ANDROID
#include "android/Log.h"
#include "android/AndroidDefine.h"
#include "android/AndroidMatrixTool.h"
#endif


#define CudaCheck(x) { cudaError_t err = x; if (err != cudaSuccess) { printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); assert(0); } }

// not implemented structs - legacy from Flex structure
struct DiffuseRenderBuffers;
struct FluidRenderer;
struct FluidRenderBuffers;
struct Texture;

struct GpuMesh
{
  GLuint mVAO;
  GLuint mTexCoordsVBO;
  GLuint mPositionsVBO;
  GLuint mNormalsVBO;
  GLuint mColorsVBO;
  GLuint mIndicesIBO;
  GLuint mTextureId;
  GLuint mNumVertices;
  GLuint mNumFaces;
  GLuint mNumIndices;
  //GLuint mSeamIndexes;
  std::vector<Vec3> positions;
  std::vector<Vec3> normals;
  std::vector<Vec4> colors;
  std::vector<Vec2> texCoordsRigid;
  std::vector<Vec4> texCoordsFilm;
  std::vector<Triangle> triangles;
  std::vector<TriangleIndexes> triangleIndexes;
  std::vector<int> triangleIntIndexes;
  //std::vector<int> seamIndexes;
  Matrix44 modelTransform;
  Vec3 center;

  Vec4 Ka, Kd, Ks;
};

/* the global Assimp scene object */
const aiScene* scene = NULL;
GLuint scene_list = 0;
aiVector3D scene_min, scene_max, scene_center;

PngImage g_chessboard_texture_image;
PngImage g_rigid_model_texture_image;
PngImage g_dynamic_texture_image;


GLuint LoadTexture(const char* filename, PngImage& image)
{
    //PngImage img;
    if (PngLoad(filename, image))
    {
        GLuint tex;

        glVerify(glGenTextures(1, &tex));
        glVerify(glActiveTexture(GL_TEXTURE0));
        glVerify(glBindTexture(GL_TEXTURE_2D, tex));

        if (1)
        {
          glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.m_width, image.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.m_data);
          //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
          //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

          glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
          glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
          

          //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
          //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
          glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER));
          glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER));
          float color[] = { 1.0f, 1.0f, 1.0f,0.2f };
          glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
          //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 7); // pick mipmap level 7 or lower
        }
        else {
          glTexStorage2D(GL_TEXTURE_2D, 7 /* mip map levels */, GL_RGB8, image.m_width, image.m_height);
          glTexSubImage2D(GL_TEXTURE_2D, 0 /* mip map level */, 0 /* xoffset */, 0 /* yoffset */, image.m_width, image.m_height, GL_RGBA, GL_UNSIGNED_BYTE, image.m_data);
          glGenerateMipmap(GL_TEXTURE_2D);

          glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR));
          glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
          glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER));
          glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER));
        
          glVerify(glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE));
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 7);
          //glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.m_width, img.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data));
        }

        // copy texture data before free memory
        //width = img.m_width;
        //height = img.m_height;
        //pixels = new GLuint[width * height * 4];
        //memcpy(pixels, img.m_data, width * height * 4);

        //PngFree(img);

        return tex;
    
    }
    else
    {
        return NULL;
    }
}


GLuint Load1DTexture(const std::vector<Vec4> textureData)
{
  GLuint tex;

  glVerify(glGenTextures(1, &tex));
  glVerify(glActiveTexture(GL_TEXTURE1));
  glVerify(glBindTexture(GL_TEXTURE_1D, tex));

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  float data[] =
  {
    0.0f, 0.0f, 1.0f, 1.0f, //blue
    0.0f, 1.0f, 1.0f, 1.0f, 
    0.0f, 1.0f, 0.0f, 1.0f, //green
    1.0f, 1.0f, 0.0f, 1.0f,
    1.0f, 0.0f, 0.0f, 1.0f //red
  };

  int textureSize = sizeof(data) / sizeof(Vec4(1.0f));

  glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA32F, textureSize, 0, GL_RGBA, GL_FLOAT, data);
  glVerify(glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP));
  glVerify(glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_T, GL_CLAMP));
  glVerify(glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
  glVerify(glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));

  return tex;
}


struct RenderTexture
{
  GLuint colorTex;
  GLuint colorFrameBuffer;

  GLuint depthTex;
  GLuint depthFrameBuffer;

  RenderTexture()
  {
    memset(this, 0, sizeof(*this));
  }
};
namespace
{
  int g_msaaSamples;
  GLuint g_msaaFbo;
  GLuint g_msaaColorBuf;
  GLuint g_msaaDepthBuf;

  int g_screenWidth;
  int g_screenHeight;

  SDL_Window* g_window;

static float g_spotMin = 0.5f;
static float g_spotMax = 1.0f;
float g_shadowBias = 0.05f;

} // anonymous namespace


extern NvFlexLibrary* g_flexLib;
extern Colour g_colors[];
extern bool g_vsync;

// shader source codes
GLchar *FILM_VERTEX_SHADER = "#version 430\n" STRINGIFY(

layout(location = 0) in vec4 aPosition;
layout(location = 1) in vec4 aNormal;
layout(location = 2) in vec4 aTexCoords;
layout(location = 3) in vec4 aColors;
uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;
uniform mat4 normalMat;
uniform bool showTexture;
out vec3 vNormal;
out vec3 vPosW;
out vec2 vTexCoords;
out vec4 vColor;

void main()
{
  vNormal = (normalize(normalMat * vec4(aNormal.xyz, 1.0))).xyz;
  vPosW = (model * vec4(aPosition.xyz, 1.0)).xyz;
  vTexCoords = showTexture ? vTexCoords = aTexCoords.xy : vec2(0.0);
  vColor = aColors;
  gl_Position = proj * view * model * vec4(aPosition.xyz, 1.0);
}
);

GLchar *FILM_FRAGMENT_SHADER = "#version 430\n" STRINGIFY(
  in vec3 vNormal;
  in vec3 vPosW;
  in vec2 vTexCoords;
  in vec4 vColor;
  uniform vec3 uLPos;
  uniform vec4 uLColor;
  uniform vec4 uColor;
  uniform vec3 uCamPos;
  uniform vec4 uAmbient;
  uniform vec4 uSpecular;
  uniform uint uSpecularExpoent;
  uniform bool showTexture;
  uniform bool showPhong;
  uniform sampler2D tex;
  out vec4 outFragColor;
  void main(void) {
    vec4 diffuse = vec4(0.0);
    vec4 specular = vec4(0.0);
    //vec4 textureBorderColor = vec4(0.0, 0.0, 0.0, 0.0);
    vec4 textureBorderColor = vec4(0.0, 0.0, 0.0, 0.0);
    // always show front facing (avoid black color showing)
    vec3 normal = gl_FrontFacing ? normalize(-vNormal) : normalize(vNormal);
    vec4 lColor = uLColor;
    // material properties
    vec4 matAmb = uAmbient;

    vec4 matDif;

    matDif = showTexture ? texture(tex, vTexCoords) : vColor;

    if (showPhong)
    {
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
    else
    {
      outFragColor = matDif;
    }
  }
);

GLchar *RIGID_VERTEX_SHADER = "#version 430\n" STRINGIFY(
  layout(location = 0) in vec3 aPosition;
  layout(location = 1) in vec3 aNormal;
  layout(location = 2) in vec2 aTexCoords;
  uniform mat4 proj;
  uniform mat4 view;
  uniform mat4 model;
  uniform mat4 normalMat;
  uniform bool showTexture;
  out vec3 vNormal;
  out vec3 vPosW;
  out vec2 vTexCoords;
  void main()
  {
    vNormal.xyz = (normalize(normalMat * vec4(aNormal, 1.0))).xyz;
    vPosW.xyz = (model * vec4(aPosition, 1.0)).xyz;
    vTexCoords = showTexture ? vTexCoords = aTexCoords : vec2(0.0);
    gl_Position = proj * view * model * vec4(aPosition, 1.0);
  }
);

GLchar *RIGID_FRAGMENT_SHADER = "#version 430\n" STRINGIFY(
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
  uniform bool showPhong;
  uniform sampler2D tex;
  out vec4 outFragColor;
  void main(void) {
    vec4 lColor = uLColor;
    //material properties
    vec4 matAmb = uAmbient;
    vec4 matDif = showTexture ? texture(tex, vTexCoords) : uColor;
    vec4 matSpec = uSpecular;
    vec4 ambient = matAmb;
    vec3 vL = normalize(uLPos - vPosW);
    vec3 normal = gl_FrontFacing ? normalize(vNormal) : normalize(-vNormal);
    float NdotL = dot(vL, normal);
    //diffuse	
    float cTheta = max(NdotL, 0.0);
    vec4 diffuse = matDif * cTheta;
    //specular
    vec3 vV = normalize(uCamPos - vPosW);
    vec3 vR = normalize(reflect(-vL, normal));
    float cOmega = max(dot(vV, vR), 0.0);
    vec4 specular = matSpec * pow(cOmega, uSpecularExpoent);
    //trick to avoid hilights when angle greater than 90
    if (NdotL < 0)
    {
      specular = vec4(0);
    }

    if (showPhong && false)
    {

      outFragColor = clamp((ambient + diffuse + specular) * lColor, 0.0, 1.0);
      //outFragColor = clamp((diffuse + specular) * lColor, 0.0, 1.0);
    }
    else
    {
      outFragColor = matDif * cTheta;
    }
  }
);

GLchar *REVERSE_TEX_VERTEX_SHADER = "#version 430\n" STRINGIFY(
  layout(location = 0) in vec4 aPosition;
layout(location = 1) in vec4 aNormal;
layout(location = 2) in vec4 aTexCoords;
layout(location = 3) in vec4 aColor;

uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;
uniform mat4 normalMat;
uniform bool showTexture;

out VS_OUT
{
  vec3 vNormal;
vec3 vPosW;
vec2 vTexCoords;
vec4 vColor;
vec2 uv0;
vec2 uv1;
} vs_out;

void main()
{
  vs_out.vNormal = (normalize(normalMat * vec4(aNormal.xyz, 1.0))).xyz;
  vs_out.vPosW = (model * vec4(aPosition.xyz, 1.0)).xyz;
  vs_out.vTexCoords = showTexture ? vs_out.vTexCoords = aTexCoords.xy : vec2(0.0);
  vs_out.vColor = aColor;
  vs_out.uv0 = fract(vs_out.vTexCoords.xy);
  vs_out.uv1 = fract(vs_out.vTexCoords.xy + vec2(0.5, 0.5)) - vec2(0.5, 0.5);

  //int indexOffset = gl_VertexID % 3;
  //int indexV0; int indexV1; int indexV2;
  //indexV0 = gl_VertexID - indexOffset + 0; 
  //indexV1 = gl_VertexID - indexOffset + 1; 
  //indexV2 = gl_VertexID - indexOffset + 2;  

  //vs_out.vertexID = gl_VertexID;

  //vs_out.triangleIndexes[0] = indexV0;
  //vs_out.triangleIndexes[1] = indexV1;
  //vs_out.triangleIndexes[2] = indexV2;

  gl_Position = proj * view * model * vec4(aPosition.xyz, 1.0);
}
//
);

GLchar *REVERSE_TEX_TESSELATION_CONTROL_SHADER = "#version 430\n" STRINGIFY(
  layout(vertices = 3) out;

//uniforms
uniform vec3 seamVertex[];
uniform uint seamVertexCount;

//inputs
in vec3 WorldPos_CS_in[];
in vec2 TexCoord_CS_in[];
in vec3 Normal_CS_in[];

//outputs
out vec3 WorldPos_ES_in[];
out vec2 TexCoord_ES_in[];
out vec3 Normal_ES_in[];

void main()
{
  // pass-through original vertices
  TexCoord_ES_in[gl_InvocationID] = TexCoord_CS_in[gl_InvocationID];
  Normal_ES_in[gl_InvocationID] = Normal_CS_in[gl_InvocationID];
  WorldPos_ES_in[gl_InvocationID] = WorldPos_CS_in[gl_InvocationID];

  // primitive setup once
  if (gl_InvocationID == 0)
  {
    gl_TessLevelInner[0] = 3;

    gl_TessLevelOuter[0] = 3;
    gl_TessLevelOuter[1] = 3;
    gl_TessLevelOuter[2] = 3;
  }

}
);

GLchar *REVERSE_TEX_TESSELATION_EVALUATION_SHADER = "#version 430\n" STRINGIFY(
  layout(triangles, equal_spacing, ccw) in;

//uniforms
uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;

//inputs
in vec3 WorldPos_ES_in[];
in vec2 TexCoord_ES_in[];
in vec3 Normal_ES_in[];

//outputs
out vec3 WorldPos_FS_in;
out vec2 TexCoord_FS_in;
out vec3 Normal_FS_in;

vec2 interpolate2D(vec2 v0, vec2 v1, vec2 v2)
{
  return vec2(gl_TessCoord.x) * v0 + vec2(gl_TessCoord.y) * v1 + vec2(gl_TessCoord.z) * v2;
}

vec3 interpolate3D(vec3 v0, vec3 v1, vec3 v2)
{
  return vec3(gl_TessCoord.x) * v0 + vec3(gl_TessCoord.y) * v1 + vec3(gl_TessCoord.z) * v2;
}

void main()
{
  // build values using barycentric coordinates
  TexCoord_FS_in = interpolate2D(TexCoord_ES_in[0], TexCoord_ES_in[1], TexCoord_ES_in[2]);
  Normal_FS_in = interpolate3D(Normal_ES_in[0], Normal_ES_in[1], Normal_ES_in[2]);
  Normal_FS_in = normalize(Normal_FS_in);
  WorldPos_FS_in = interpolate3D(WorldPos_ES_in[0], WorldPos_ES_in[1], WorldPos_ES_in[2]);

  gl_Position = proj * view * model * vec4(WorldPos_FS_in, 1.0);
}
);

GLchar *REVERSE_TEX_FRAGMENT_SHADER = "#version 430\n" STRINGIFY(
  uniform vec3 uLPos;
uniform vec4 uLColor;
uniform vec4 uColor;
uniform vec3 uCamPos;
uniform vec4 uAmbient;
uniform vec4 uSpecular;
uniform uint uSpecularExpoent;
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
  vec4 diffuse = vec4(0.0);
  vec4 specular = vec4(0.0);
  vec4 textureBorderColor = vec4(0.0, 0.0, 0.0, 0.0);
  // always show front facing (avoid black color showing)
  vec3 normal = gl_FrontFacing ? normalize(-fs_in.vNormal) : normalize(fs_in.vNormal);
  vec4 lColor = uLColor;
  // material properties
  vec4 matAmb = uAmbient;

  vec4 matDif;

  //vec2 uvT; //Tarini 
  //uvT.x = ( fwidth( fs_in.uv0.x ) < fwidth( fs_in.uv1.x )-0.001 )? fs_in.uv0.x : fs_in.uv1.x;
  //uvT.y = ( fwidth( fs_in.uv0.y ) < fwidth( fs_in.uv1.y )-0.001 )? fs_in.uv0.y : fs_in.uv1.y;

  //uvT.x = ( fwidth( fs_in.uv0.x ) < fwidth( fs_in.uv0.x +0.001) )? fs_in.uv0.x : fs_in.uv0.x +000.1;
  //uvT.y = ( fwidth( fs_in.uv0.y ) < fwidth( fs_in.uv0.y +0.001) )? fs_in.uv0.y : fs_in.uv0.y +000.1;

  matDif = showTexture ? texture(tex, fs_in.vTexCoords) : fs_in.vColor;
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
  outFragColor = matDif;
}
//
);

GLchar *REVERSE_TEX_GEOMETRY_SHADER = "#version 430\n" STRINGIFY(
  layout(triangles) in;
  layout(triangle_strip, max_vertices = 36) out;

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

    ComputeTexDistance(gs_in[0].vTexCoords, gs_in[1].vTexCoords, gs_in[2].vTexCoords, texDistance01, texDistance02, texDistance12);

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
);


GLchar *REVERSE_TEX_GEOMETRY_SHADER_V2 = "#version 430\n" STRINGIFY(
  layout(triangles) in;
layout(triangle_strip, max_vertices = 36) out;
uniform float uMaxDistanceUV;
uniform float uNearDistanceUV;
uniform float uWeight1;
uniform float uWeight2;
uniform sampler2D tex;

in VS_OUT
{
  vec3 vNormal;
vec3 vPosW;
vec2 vTexCoords;
vec4 vColor;
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
  gs_out.vTexCoords = texCoordFrom + textDirection * epsilon;
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
//
);

GLchar *REVERSE_TEX_GEOMETRY_SHADER_V3 = "#version 430\n" STRINGIFY(
  layout(triangles) in; \n
  layout(triangle_strip, max_vertices = 36) out; \n
  uniform float uMaxDistanceUV; \n
  uniform float uNearDistanceUV; \n
  uniform float uWeight1; \n
  uniform float uWeight2; \n
  uniform sampler2D tex; \n
  uniform sampler2D heatmap; \n
  \n
  in VS_OUT\n
{ \n
vec3 vNormal; \n
vec3 vPosW; \n
vec2 vTexCoords; \n
} gs_in[]; \n
\n
out GS_OUT\n
{ \n
vec3 vNormal; \n
vec3 vPosW; \n
vec2 vTexCoords; \n
} gs_out; \n
\n
void EmitTextureVertex(int index)\n
{ \n
gl_Position = gl_in[index].gl_Position; \n
gs_out.vNormal = gs_in[index].vNormal; \n
gs_out.vTexCoords = gs_in[index].vTexCoords; \n
gs_out.vPosW = gs_in[index].vPosW; \n
EmitVertex(); \n
}\n
\n
void EmitWeightedTexVertex(int index1, int index2, float weight1, float weight2)\n
{ \n
gl_Position = 0.5f * (gl_in[index1].gl_Position + gl_in[index2].gl_Position); \n
gs_out.vNormal = gs_in[index1].vNormal; // the mesh is always a plane assumes always the same normal\n
gs_out.vTexCoords = (weight1 * gs_in[index1].vTexCoords + weight2 * gs_in[index2].vTexCoords); // vertex with greater texture weight\n
gs_out.vPosW = 0.5 * (gs_in[index1].vPosW + gs_in[index2].vPosW); \n
EmitVertex(); \n
}\n
\n
void EmitWeightedTexVertexCenter(int index1, int index2, vec2 texCoord1, vec2 texCoord2, vec2 texCoord3, float epsilon)\n
{ \n
gl_Position = 0.5f * (gl_in[index1].gl_Position + gl_in[index2].gl_Position); \n
gs_out.vNormal = gs_in[index1].vNormal; // the mesh is always a plane assumes always the same normal\n
\n
vec2 centerTexCoord = 0.33f * (texCoord1 + texCoord2 + texCoord3); \n
vec2 centerTexDirection = normalize(centerTexCoord - texCoord1); \n
\n
gs_out.vTexCoords = texCoord1 + centerTexDirection * epsilon; \n
gs_out.vPosW = 0.5 * (gs_in[index1].vPosW + gs_in[index2].vPosW); \n
EmitVertex(); \n
}\n
\n
void EmitTexVertexByNearColor(int index1, int index2, vec2 texCoords)\n
{ \n
gl_Position = 0.5f * (gl_in[index1].gl_Position + gl_in[index2].gl_Position); \n
gs_out.vNormal = gs_in[index1].vNormal; // the mesh is always a plane assumes always the same normal\n
gs_out.vTexCoords = texCoords; \n
gs_out.vPosW = 0.5 * (gs_in[index1].vPosW + gs_in[index2].vPosW); \n
EmitVertex(); \n
}\n
\n
void EmitStrips(vec3 farVertexIndex, vec3 nearVertex1, vec3 nearVertex2, vec2 texFarVertex, vec2 texNearVertex1, vec2 texNearVertex2, float w1, float w2, int level)\n
{ \n
// v0 cross the seam\n
// strip 1\n
level++;

EmitTextureVertex(nearVertex1); \n
EmitWeightedTexVertex(farVertexIndex, nearVertex1, w1, w2); \n
EmitWeightedTexVertex(nearVertex1, nearVertex2, 0.5f, 0.5f); \n
EmitWeightedTexVertex(farVertexIndex, nearVertex2, w1, w2); \n
EmitTextureVertex(nearVertex2); \n
\n
EndPrimitive(); \n
\n
// strip 2\n
EmitWeightedTexVertex(nearVertex1, farVertexIndex, w1, w2); \n
EmitTextureVertex(farVertexIndex); \n
EmitWeightedTexVertex(nearVertex2, farVertexIndex, w1, w2); \n
\n
EndPrimitive(); \n
\n

}\n
\n
bool isSeam(float maxDistanceUV, bool cross01, bool cross02, bool cross12, bool crossAll)\n
{ \n
return (cross01 || cross02 || cross12 || crossAll); \n
}\n
\n
bool v0_CrossTheSeam(bool cross01, bool cross02)\n
{ \n
return (cross01 && cross02); \n
}\n

bool v1_CrossTheSeam(bool cross01, bool cross12)\n
{ \n
return (cross01 && cross12); \n
}\n
\n
bool v2_CrossTheSeam(bool cross02, bool cross12)\n
{ \n
return (cross02 && cross12); \n
}\n
\n
bool all_CrossTheSeam(float maxDistanceUV, float distance01, float distance02, float distance12)\n
{ \n
float crossAllMaxDistanceUV = 0.33f * maxDistanceUV; \n
\n
return (distance01 > crossAllMaxDistanceUV && distance02 > crossAllMaxDistanceUV && distance12 > crossAllMaxDistanceUV);
}\n
\n
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
  vec4 texColor = texture(tex, texCoords);

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

  int farVertexIndex; int nearVertex1; int nearVertex2;
  float texDistance01; float texDistance02; float texDistance12;
  bool cross01; bool cross02; bool cross12; \n
    int level = 0; \n

    vec2 halfTex01; vec2 halfTex02; vec2 halfTex12;
  vec4 half01; vec4 half02; vec4 half12;

  ComputeHalfTextures(gs_in[0].vTexCoords, gs_in[1].vTexCoords, gs_in[2].vTexCoords, halfTex01, halfTex02, halfTex12);
  ComputeHalfDistances(gl_in[0].gl_Position, gl_in[1].gl_Position, gl_in[2].gl_Position, half01, half02, half12);

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
      EmitWeightedTexVertex(1, 0, .9f, .1f);
      EmitWeightedTexVertex(1, 2, .9f, .1f);

      EndPrimitive();

      // strip 2
      EmitWeightedTexVertex(2, 1, .9f, .1f);
      EmitWeightedTexVertex(2, 0, .9f, .1f);
      EmitTextureVertex(2);

      EndPrimitive();

      // strip 3
      EmitWeightedTexVertex(0, 1, .9f, .1f);
      EmitTextureVertex(0);
      EmitWeightedTexVertex(0, 2, .9f, .1f);

      EndPrimitive();

      // strip 4 center  
      EmitWeightedTexVertex(0, 1, .5f, .5f);
      EmitWeightedTexVertex(0, 2, .5f, .5f);
      EmitWeightedTexVertex(1, 2, .5f, .5f);

      EndPrimitive();

    }
    else if (v0_CrossTheSeam(cross01, cross02))
    {
      farVertexIndex = 0; nearVertex1 = 1; nearVertex2 = 2;

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, w1, w2, level);

    }
    else if (v1_CrossTheSeam(cross01, cross12))
    {
      farVertexIndex = 1; nearVertex1 = 2; nearVertex2 = 0;

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, w1, w2, level);

    }
    else if (v2_CrossTheSeam(cross02, cross12))
    {
      farVertexIndex = 2; nearVertex1 = 0; nearVertex2 = 1;

      EmitStrips(farVertexIndex, nearVertex1, nearVertex2, w1, w2, level);

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
);


namespace OGL_Renderer
{

  void InitRender(const RenderInitOptions& options)
  {
	  SDL_Window* window = options.window;
	  int msaaSamples = options.numMsaaSamples;

	  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

	  //SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);

	  // Turn on double buffering with a 24bit Z buffer.
	  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

	  SDL_GL_CreateContext(window);

	  // This makes our buffer swap syncronized with the monitor's vertical refresh
	  SDL_GL_SetSwapInterval(g_vsync);

	  if (!gladLoadGLLoader(SDL_GL_GetProcAddress))
	  {
		  printf("Could not initialize GL extensions\n");
	  }

	  imguiRenderGLInit(GetFilePathByPlatform("../../data/DroidSans.ttf").c_str());


    if (s_rigidBodyProgram == GLuint(-1))
    {
      Shader rigidBodyShaders[5] = {
        { "RIGID_VERTEX_SHADER", GL_VERTEX_SHADER, RIGID_VERTEX_SHADER },
        { "RIGID_FRAGMENT_SHADER", GL_FRAGMENT_SHADER, RIGID_FRAGMENT_SHADER },
        { NULL, GL_TESS_CONTROL_SHADER, NULL },
        { NULL, GL_TESS_EVALUATION_SHADER, NULL },
        { NULL, GL_GEOMETRY_SHADER, NULL }
      };

      s_rigidBodyProgram = InitShader(rigidBodyShaders);
    }

    if (s_filmProgram == GLuint(-1))
    {
      Shader filmShaders[5] = {
        { "FILM_VERTEX_SHADER", GL_VERTEX_SHADER, FILM_VERTEX_SHADER },
        { "FILM_FRAGMENT_SHADER", GL_FRAGMENT_SHADER, RIGID_FRAGMENT_SHADER },
        { NULL, GL_TESS_CONTROL_SHADER, NULL },
        { NULL, GL_TESS_EVALUATION_SHADER, NULL },
        { NULL, GL_GEOMETRY_SHADER, NULL }
      };
      s_filmProgram = InitShader(filmShaders);
    }

    if (s_reverseTexProgram == GLuint(-1))
    {
      Shader reverseTextureShaders[5] = {
        { "../../shaders/reverse_tex.vs", GL_VERTEX_SHADER, NULL },
        { "../../shaders/reverse_tex.fs", GL_FRAGMENT_SHADER, NULL },
        { "../../shaders/reverse_tex_v2.tcs", GL_TESS_CONTROL_SHADER, NULL },
        { "../../shaders/reverse_tex_v2.tes", GL_TESS_EVALUATION_SHADER, NULL },
        { "../../shaders/reverse_tex_v2.gs", GL_GEOMETRY_SHADER, NULL }
      };

      //disable tesselation shaders
      if (1) {
        reverseTextureShaders[2].filename = NULL;
        reverseTextureShaders[2].source= NULL;
        reverseTextureShaders[3].filename = NULL;
        reverseTextureShaders[3].source = NULL;
      }
      else {
        reverseTextureShaders[4].filename = "../../shaders/pass_through.gs";
        reverseTextureShaders[4].source = NULL;
        //
        GLint MaxPatchVertices = 0;
        glGetIntegerv(GL_MAX_PATCH_VERTICES, &MaxPatchVertices);
        printf("Tesselation Shader - Max supported patch vertices %d\n", MaxPatchVertices);
        glPatchParameteri(GL_PATCH_VERTICES, 3);
      }

      g_enable_paches = reverseTextureShaders[2].filename != NULL;
      s_reverseTexProgram = InitShader(reverseTextureShaders, true);
    }

    g_chessboard_texture_id = LoadTexture(GetFilePathByPlatform("../../textures/malha_rgb.jpg").c_str(), g_chessboard_texture_image);

    std::vector<Vec4> heatmapColors;
    heatmapColors.push_back(Vec4(0.0f, 0.0f, 1.0f, 1.0f));
    heatmapColors.push_back(Vec4(0.0f, 1.0f, 1.0f, 1.0f));
    heatmapColors.push_back(Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    heatmapColors.push_back(Vec4(1.0f, 1.0f, 0.0f, 1.0f));
    heatmapColors.push_back(Vec4(1.0f, 0.0f, 0.0f, 1.0f));

    g_heatmap_texture_id = Load1DTexture(heatmapColors);

	  g_msaaSamples = msaaSamples;
	  g_window = window;
  }

  void DestroyRender()
  {

  }

  void StartFrame(Vec4 clearColor)
  {
  	glEnable(GL_DEPTH_TEST);
  	glEnable(GL_CULL_FACE);
  	glDisable(GL_LIGHTING);
  	glDisable(GL_BLEND);
  
  	glPointSize(5.0f);
  
  	glVerify(glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, g_msaaFbo));
  	glVerify(glClearColor(SKY_COLOR.x, SKY_COLOR.y, SKY_COLOR.z, SKY_COLOR.w));
  	glVerify(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));


  }

  void EndFrame()
  {
    if (g_msaaFbo)
    {
      // blit the msaa buffer to the window
      glVerify(glBindFramebuffer(GL_READ_FRAMEBUFFER_EXT, g_msaaFbo));
      glVerify(glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, 0));
      glVerify(glBlitFramebuffer(0, 0, g_screenWidth, g_screenHeight, 0, 0, g_screenWidth, g_screenHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR));
    }
  
      // render help to back buffer
    glVerify(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    glVerify(glClear(GL_DEPTH_BUFFER_BIT));
    
  }

  void SetView(Matrix44 view, Matrix44 proj)
  {
  	glMatrixMode(GL_PROJECTION);
  	glLoadMatrixf(proj);
  
  	glMatrixMode(GL_MODELVIEW);
  	glLoadMatrixf(view);	
  }
  
  void SetFillMode(bool wireframe)
  {
  	glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);
  }
  
  void SetCullMode(bool enabled)
  {
  	if (enabled)
  		glEnable(GL_CULL_FACE);
  	else
  		glDisable(GL_CULL_FACE);
  }


void imguiGraphDraw()
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glActiveTexture(GL_TEXTURE0);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);
	glActiveTexture(GL_TEXTURE1);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE2);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE3);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE4);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_TEXTURE_CUBE_MAP);
	glActiveTexture(GL_TEXTURE5);
	glDisable(GL_TEXTURE_2D);

	glActiveTexture(GL_TEXTURE0);

	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glDisable(GL_POINT_SPRITE);

	// save scene camera transform
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	const Matrix44 ortho = OrthographicMatrix(0.0f, float(g_screenWidth), 0.0f, float(g_screenHeight), -1.0f, 1.0f);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadMatrixf(ortho);

	glUseProgram(0);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_TEXTURE_2D);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	imguiRenderGLDraw();

	// restore camera transform (for picking)
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

void ReshapeRender(int width, int height, bool minimized)
{
	if (g_msaaSamples)
	{
		glVerify(glBindFramebuffer(GL_FRAMEBUFFER, 0));

		if (g_msaaFbo)
		{
			glVerify(glDeleteFramebuffers(1, &g_msaaFbo));
			glVerify(glDeleteRenderbuffers(1, &g_msaaColorBuf));
			glVerify(glDeleteRenderbuffers(1, &g_msaaDepthBuf));
		}

		int samples;
		glGetIntegerv(GL_MAX_SAMPLES_EXT, &samples);

		// clamp samples to 4 to avoid problems with point sprite scaling
		samples = Min(samples, Min(g_msaaSamples, 4));

		glVerify(glGenFramebuffers(1, &g_msaaFbo));
		glVerify(glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo));

		glVerify(glGenRenderbuffers(1, &g_msaaColorBuf));
		glVerify(glBindRenderbuffer(GL_RENDERBUFFER, g_msaaColorBuf));
		glVerify(glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, width, height));

		glVerify(glGenRenderbuffers(1, &g_msaaDepthBuf));
		glVerify(glBindRenderbuffer(GL_RENDERBUFFER, g_msaaDepthBuf));
		glVerify(glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT, width, height));
		glVerify(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, g_msaaDepthBuf));

		glVerify(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, g_msaaColorBuf));

		glVerify(glCheckFramebufferStatus(GL_FRAMEBUFFER));

		glEnable(GL_MULTISAMPLE);
	}

	g_screenWidth = width;
	g_screenHeight = height;
}

void GetViewRay(int x, int y, Vec3& origin, Vec3& dir)
{
	float modelview[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

	float projection[16];
	glGetFloatv(GL_PROJECTION_MATRIX, projection);

	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	float nearPos[3];
	UnProjectf(float(x), float(y), 0.0f, modelview, projection, viewport, nearPos);

	float farPos[3];
	UnProjectf(float(x), float(y), 1.0f, modelview, projection, viewport, farPos);

	origin = Vec3(float(nearPos[0]), float(nearPos[1]), float(nearPos[2]));
	dir = Normalize(Vec3(float(farPos[0]-nearPos[0]), float(farPos[1]-nearPos[1]), float(farPos[2]-nearPos[2])));
}

Vec3 GetScreenCoord(Vec3& pos) {
	float modelview[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

	float projection[16];
	glGetFloatv(GL_PROJECTION_MATRIX, projection);

	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	float screen[3];
	Projectf(pos.x, pos.y, pos.z, modelview, projection, viewport, screen);

	return Vec3((float)screen[0], (float)screen[1], (float)screen[2]);
}

void ReadFrame(int* backbuffer, int width, int height)
{
	glVerify(glReadBuffer(GL_BACK));
	glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, backbuffer);
}

void PresentFrame(bool fullsync)
{
#ifndef ANDROID
	SDL_GL_SetSwapInterval(fullsync);
  glFinish();
	SDL_GL_SwapWindow(g_window);
#endif
}

RenderTexture* CreateRenderTarget(int width, int height, bool depth)
{
	return NULL;
}

void DestroyRenderTexture(RenderTexture* t)
{
	if (t)
	{
		if (t->colorTex)
			glDeleteTextures(1, &t->colorTex);

		if (t->colorFrameBuffer)
			glDeleteFramebuffers(1, &t->colorFrameBuffer);

		if (t->depthTex)
			glDeleteTextures(1, &t->depthTex);

		if (t->depthFrameBuffer)
			glDeleteFramebuffers(1, &t->depthFrameBuffer);

		delete t;

		
	}
}


  // fixes some banding artifacts with repeated blending during thickness and diffuse rendering
#define USE_HDR_DIFFUSE_BLEND 0




#ifdef ANDROID
  void ResetProgramId()
  {
    s_diffuseProgram = GLuint(-1);
    s_shadowProgram = GLuint(-1);
  }
#endif

  GpuMesh* CreateGpuMesh(const Mesh* m)
  {
  	GpuMesh* mesh = new GpuMesh();
  
  	mesh->mNumVertices = m->GetNumVertices();
  	mesh->mNumFaces = m->GetNumFaces();
  
    // vbos
    glVerify(glGenBuffers(1, &mesh->mPositionsVBO));
  	glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO));
  	glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * m->m_positions.size(), &m->m_positions[0], GL_STATIC_DRAW));
  
  	glVerify(glGenBuffers(1, &mesh->mNormalsVBO));
  	glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mNormalsVBO));
  	glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * m->m_normals.size(), &m->m_normals[0], GL_STATIC_DRAW));
  
  	glVerify(glGenBuffers(1, &mesh->mIndicesIBO));
  	glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->mIndicesIBO));
  	glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * m->m_indices.size(), &m->m_indices[0], GL_STATIC_DRAW));
  	glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
  
  	return mesh;
  }

  void DestroyGpuMesh(GpuMesh* m)
  {
    glVerify(glDeleteVertexArrays(1, &m->mVAO));
  	glVerify(glDeleteBuffers(1, &m->mPositionsVBO));
  	glVerify(glDeleteBuffers(1, &m->mNormalsVBO));
  	glVerify(glDeleteBuffers(1, &m->mIndicesIBO));
    glVerify(glDeleteBuffers(1, &m->mColorsVBO));
    glVerify(glDeleteBuffers(1, &m->mTexCoordsVBO));
  }

  void DrawGpuMesh(GpuMesh* m, const Matrix44& xform, const Vec3& color)
  {
  	if (m)
  	{
  		GLint program;
  		glGetIntegerv(GL_CURRENT_PROGRAM, &program);
  
  		if (program)
  			glUniformMatrix4fv(glGetUniformLocation(program, "objectTransform"), 1, false, xform);
  
  		glVerify(glColor3fv(color));
  		glVerify(glSecondaryColor3fv(color));
  
  		glVerify(glEnableClientState(GL_VERTEX_ARRAY));
  		glVerify(glBindBuffer(GL_ARRAY_BUFFER, m->mPositionsVBO));
  		glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float) * 3, 0));
  
  		glVerify(glEnableClientState(GL_NORMAL_ARRAY));
  		glVerify(glBindBuffer(GL_ARRAY_BUFFER, m->mNormalsVBO));
  		glVerify(glNormalPointer(GL_FLOAT, sizeof(float) * 3, 0));
  
  		glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->mIndicesIBO));
  
  		glVerify(glDrawElements(GL_TRIANGLES, m->mNumFaces * 3, GL_UNSIGNED_INT, 0));
  
  		glVerify(glDisableClientState(GL_VERTEX_ARRAY));
  		glVerify(glDisableClientState(GL_NORMAL_ARRAY));
  
  		glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
  		glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
  
  		if (program)
  			glUniformMatrix4fv(glGetUniformLocation(program, "objectTransform"), 1, false, Matrix44::kIdentity);
  	}
  }

  void DrawGpuMeshInstances(GpuMesh* m, const Matrix44* xforms, int n, const Vec3& color)
  {
    if (m)
    {
      GLint program;
      glGetIntegerv(GL_CURRENT_PROGRAM, &program);

      GLint param = glGetUniformLocation(program, "objectTransform");

      glVerify(glColor3fv(color));
      glVerify(glSecondaryColor3fv(color));

      glVerify(glEnableClientState(GL_VERTEX_ARRAY));
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, m->mPositionsVBO));
      glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float) * 3, 0));

      glVerify(glEnableClientState(GL_NORMAL_ARRAY));
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, m->mNormalsVBO));
      glVerify(glNormalPointer(GL_FLOAT, sizeof(float) * 3, 0));

      glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->mIndicesIBO));

      for (int i = 0; i < n; ++i)
      {
        if (program)
          glUniformMatrix4fv(param, 1, false, xforms[i]);

        glVerify(glDrawElements(GL_TRIANGLES, m->mNumFaces * 3, GL_UNSIGNED_INT, 0));
      }

      glVerify(glDisableClientState(GL_VERTEX_ARRAY));
      glVerify(glDisableClientState(GL_NORMAL_ARRAY));

      glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    }
  }

  void BeginLines()
  {
    glUseProgram(0);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    glLineWidth(2.5f);

    for (int i = 0; i < 8; ++i)
    {
      glActiveTexture(GL_TEXTURE0 + i);
      glDisable(GL_TEXTURE_2D);
    }

    glBegin(GL_LINES);
  }

  void DrawLine(const Vec3& p, const Vec3& q, const Vec4& color)
  {
    glColor4fv(color);
    glVertex3fv(p);
    glVertex3fv(q);
  }

  void EndLines()
  {
    glEnd();
  }

  void BeginPoints(float size)
  {
    glPointSize(size);

    glUseProgram(0);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    for (int i = 0; i < 8; ++i)
    {
      glActiveTexture(GL_TEXTURE0 + i);
      glDisable(GL_TEXTURE_2D);
    }

    glBegin(GL_POINTS);
  }

  void DrawPoint(const Vec3& p, const Vec4& color)
  {
    glColor3fv(color);
    glVertex3fv(p);
  }

  void EndPoints()
  {
    glEnd();
  }

  float SyncAndGetRenderTime(unsigned long long* begin, unsigned long long* end, unsigned long long* freq)
  {
    *begin = 0;
    *end = 0;
    *freq = 1;
    return 0.0f;
  }

  float RendererGetDeviceTimestamps(unsigned long long* begin, unsigned long long* end, unsigned long long* freq) { return 0.0f; }
  void* GetGraphicsCommandQueue() { return nullptr; }
  void GraphicsTimerBegin() { }
  void GraphicsTimerEnd() { }

  void StartGpuWork() { }
  void EndGpuWork() { }

  void GetRenderDevice(void** deviceOut, void** contextOut)
  {
    *deviceOut = nullptr;
    *contextOut = nullptr;
  }

  void DrawImguiGraph()
  {
    imguiGraphDraw();
  }

  // not implemented - legacy from Flex - only bypass the methods for binary compatibility with the original Flex interface
  void DrawCloth(const Vec4* positions, const Vec4* normals, const float* uvs, const int* indices, int numTris, int numPositions, int colorIndex, float expand, bool twosided, bool smooth) {}
  void DrawPlanes(Vec4* planes, int n, float bias) {}
  void DrawPoints(FluidRenderBuffers* buffersIn, int n, int offset, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ShadowMap* shadowMap, bool showDensity) {}
  void DrawMesh(const Mesh* m, Vec3 color) {}
  void BindSolidShader(Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ShadowMap* shadowMap, float bias, Vec4 fogColor) {}
  void UnbindSolidShader() {}
  ShadowMap* ShadowCreate() { return nullptr; }
  void ShadowEnd() {}
  void SetMaterial(const Matrix44& xform, const RenderMaterial& mat) {}
  void ShadowDestroy(ShadowMap* map) {}
  void ShadowBegin(ShadowMap* map) {}
  void DrawRope(Vec4* positions, int* indices, int numIndices, float radius, int color) {}
  void BindHydrographicShader(Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ShadowMap* shadowMap, float bias, Vec4 fogColor) {}
  DiffuseRenderBuffers* CreateDiffuseRenderBuffers(int numDiffuseParticles, bool& enableInterop) { return nullptr; }
  void DestroyFluidRenderer(FluidRenderer* renderer) {}
  FluidRenderBuffers* CreateFluidRenderBuffers(int numFluidParticles, bool enableInterop) { return nullptr; }
  void UpdateFluidRenderBuffers(FluidRenderBuffers* buffersIn, NvFlexSolver* solver, bool anisotropy, bool density) {}
  FluidRenderer* CreateFluidRenderer(uint32_t width, uint32_t height) { return nullptr; }
  void UpdateFluidRenderBuffers(FluidRenderBuffers* buffersIn, Vec4* particles, float* densities, Vec4* anisotropy1, Vec4* anisotropy2, Vec4* anisotropy3, int numParticles, int* indices, int numIndices) {}
  void DestroyFluidRenderBuffers(FluidRenderBuffers* buffers) {}
  void DestroyDiffuseRenderBuffers(DiffuseRenderBuffers* buffersIn) {}
  void UpdateDiffuseRenderBuffers(DiffuseRenderBuffers* buffersIn, NvFlexSolver* solver) {}
  void UpdateDiffuseRenderBuffers(DiffuseRenderBuffers* buffersIn, Vec4* diffusePositions, Vec4* diffuseVelocities, int numDiffuseParticles) {}

}	// OGL Renderer


#include "../source/demoContext.h"
#include "demoContextOGL.h"

DemoContext* CreateDemoContextOGL()
{
  return new DemoContextOGL;
}

bool DemoContextOGL::initialize(const RenderInitOptions& options)
{
  OGL_Renderer::InitRender(options);
  return true;
}

void DemoContextOGL::startFrame(Vec4 colorIn)
{
  OGL_Renderer::StartFrame(colorIn);
}

void DemoContextOGL::endFrame()
{
  OGL_Renderer::EndFrame();
}

void DemoContextOGL::presentFrame(bool fullsync)
{
  OGL_Renderer::PresentFrame(fullsync);
}

void DemoContextOGL::readFrame(int* buffer, int width, int height)
{
  OGL_Renderer::ReadFrame(buffer, width, height);
}

void DemoContextOGL::getViewRay(int x, int y, Vec3& origin, Vec3& dir)
{
  OGL_Renderer::GetViewRay(x, y, origin, dir);
}

void DemoContextOGL::setView(Matrix44 view, Matrix44 projection)
{
  OGL_Renderer::SetView(view, projection);
}

void DemoContextOGL::renderEllipsoids(FluidRenderer* renderer, FluidRenderBuffers* buffers, int n, int offset, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ::ShadowMap* shadowMap, Vec4 color, float blur, float ior, bool debug)
{
	std::cout << "Not implemented yet" << std::endl;
}

void DemoContextOGL::drawMesh(const Mesh* m, Vec3 color)
{
  OGL_Renderer::DrawMesh(m, color);
}

void DemoContextOGL::drawCloth(const Vec4* positions, const Vec4* normals, const float* uvs, const int* indices, int numTris, int numPositions, int colorIndex, float expand, bool twosided, bool smooth)
{
  OGL_Renderer::DrawCloth(positions, normals, uvs, indices, numTris, numPositions, colorIndex, expand, twosided, smooth);
}

void DemoContextOGL::drawRope(Vec4* positions, int* indices, int numIndices, float radius, int color)
{
  OGL_Renderer::DrawRope(positions, indices, numIndices, radius, color);
}

void DemoContextOGL::drawPlane(const Vec4& p, bool color)
{
  OGL_Renderer::DrawPlane(p, color);
}

void DemoContextOGL::drawPlanes(Vec4* planes, int n, float bias)
{
  OGL_Renderer::DrawPlanes(planes, n, bias);
}

void DemoContextOGL::drawPoints(FluidRenderBuffers* buffers, int n, int offset, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ::ShadowMap* shadowTex, bool showDensity)
{
  OGL_Renderer::DrawPoints(buffers, n, offset, radius, screenWidth, screenAspect, fov, lightPos, lightTarget, lightTransform, shadowTex, showDensity);
}

void DemoContextOGL::graphicsTimerBegin()
{
  OGL_Renderer::GraphicsTimerBegin();
}

void DemoContextOGL::graphicsTimerEnd()
{
  OGL_Renderer::GraphicsTimerEnd();
}

float DemoContextOGL::rendererGetDeviceTimestamps(unsigned long long* begin, unsigned long long* end, unsigned long long* freq)
{
  return OGL_Renderer::RendererGetDeviceTimestamps(begin, end, freq);
}

void DemoContextOGL::bindSolidShader(Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ::ShadowMap* shadowMap, float bias, Vec4 fogColor)
{
  OGL_Renderer::BindSolidShader(lightPos, lightTarget, lightTransform, shadowMap, bias, fogColor);
}

void DemoContextOGL::unbindSolidShader()
{
  OGL_Renderer::UnbindSolidShader();
}

ShadowMap* DemoContextOGL::shadowCreate()
{
  return OGL_Renderer::ShadowCreate();
}

void DemoContextOGL::shadowDestroy(ShadowMap* map)
{
  OGL_Renderer::ShadowDestroy(map);
}

void DemoContextOGL::shadowBegin(ShadowMap* map)
{
  OGL_Renderer::ShadowBegin(map);
}

void DemoContextOGL::shadowEnd()
{
  OGL_Renderer::ShadowEnd();
}

FluidRenderer* DemoContextOGL::createFluidRenderer(uint32_t width, uint32_t height)
{
  return OGL_Renderer::CreateFluidRenderer(width, height);
}

void DemoContextOGL::destroyFluidRenderer(FluidRenderer* renderer)
{
  OGL_Renderer::DestroyFluidRenderer(renderer);
}

FluidRenderBuffers* DemoContextOGL::createFluidRenderBuffers(int numParticles, bool enableInterop)
{
  return OGL_Renderer::CreateFluidRenderBuffers(numParticles, enableInterop);
}

void DemoContextOGL::updateFluidRenderBuffers(FluidRenderBuffers* buffers, NvFlexSolver* flex, bool anisotropy, bool density)
{
  OGL_Renderer::UpdateFluidRenderBuffers(buffers, flex, anisotropy, density);
}

void DemoContextOGL::updateFluidRenderBuffers(FluidRenderBuffers* buffers, Vec4* particles, float* densities, Vec4* anisotropy1, Vec4* anisotropy2, Vec4* anisotropy3, int numParticles, int* indices, int numIndices)
{
  OGL_Renderer::UpdateFluidRenderBuffers(buffers, particles, densities, anisotropy1, anisotropy2, anisotropy3, numParticles, indices, numIndices);
}

void DemoContextOGL::destroyFluidRenderBuffers(FluidRenderBuffers* buffers)
{
  OGL_Renderer::DestroyFluidRenderBuffers(buffers);
}

GpuMesh* DemoContextOGL::createGpuMesh(const Mesh* m)
{
  return OGL_Renderer::CreateGpuMesh(m);
}

void DemoContextOGL::destroyGpuMesh(GpuMesh* mesh)
{
  OGL_Renderer::DestroyGpuMesh(mesh);
}

void DemoContextOGL::drawGpuMesh(GpuMesh* m, const Matrix44& xform, const Vec3& color)
{
  OGL_Renderer::DrawGpuMesh(m, xform, color);
}

void DemoContextOGL::drawGpuMeshInstances(GpuMesh* m, const Matrix44* xforms, int n, const Vec3& color)
{
  OGL_Renderer::DrawGpuMeshInstances(m, xforms, n, color);
}

DiffuseRenderBuffers* DemoContextOGL::createDiffuseRenderBuffers(int numDiffuseParticles, bool& enableInterop)
{
  return OGL_Renderer::CreateDiffuseRenderBuffers(numDiffuseParticles, enableInterop);
}

void DemoContextOGL::destroyDiffuseRenderBuffers(DiffuseRenderBuffers* buffers)
{
  OGL_Renderer::DestroyDiffuseRenderBuffers(buffers);
}

void DemoContextOGL::updateDiffuseRenderBuffers(DiffuseRenderBuffers* buffers, Vec4* diffusePositions, Vec4* diffuseVelocities, int numDiffuseParticles)
{
  OGL_Renderer::UpdateDiffuseRenderBuffers(buffers, diffusePositions, diffuseVelocities, numDiffuseParticles);
}

void DemoContextOGL::updateDiffuseRenderBuffers(DiffuseRenderBuffers* buffers, NvFlexSolver* solver)
{
  OGL_Renderer::UpdateDiffuseRenderBuffers(buffers, solver);
}

void DemoContextOGL::drawDiffuse(FluidRenderer* render, const DiffuseRenderBuffers* buffers, int n, float radius, float screenWidth, float screenAspect, float fov, Vec4 color, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ::ShadowMap* shadowMap, float motionBlur, float inscatter, float outscatter, bool shadowEnabled, bool front)
{
	std::cout << "Not implemented yet" << std::endl;
}

int DemoContextOGL::getNumDiffuseRenderParticles(DiffuseRenderBuffers* buffers)
{
	std::cout << "Not implemented yet" << std::endl;
	return NULL;
}

void DemoContextOGL::beginLines()
{
  OGL_Renderer::BeginLines();
}

void DemoContextOGL::drawLine(const Vec3& p, const Vec3& q, const Vec4& color)
{
  OGL_Renderer::DrawLine(p, q, color);
}

void DemoContextOGL::endLines()
{
  OGL_Renderer::EndLines();
}

void DemoContextOGL::beginPoints(float size)
{
  OGL_Renderer::BeginPoints(size);
}

void DemoContextOGL::drawPoint(const Vec3& p, const Vec4& color)
{
  OGL_Renderer::DrawPoint(p, color);
}

void DemoContextOGL::endPoints()
{
  OGL_Renderer::EndPoints();
}

void DemoContextOGL::onSizeChanged(int width, int height, bool minimized)
{
  OGL_Renderer::ReshapeRender(width, height, minimized);
}

void DemoContextOGL::startGpuWork()
{
  OGL_Renderer::StartGpuWork();
}

void DemoContextOGL::endGpuWork()
{
  OGL_Renderer::EndGpuWork();
}

void DemoContextOGL::flushGraphicsAndWait()
{
}

void DemoContextOGL::setFillMode(bool wire)
{
  OGL_Renderer::SetFillMode(wire);
}

void DemoContextOGL::setCullMode(bool enabled)
{
  OGL_Renderer::SetCullMode(enabled);
}

void DemoContextOGL::drawImguiGraph()
{
  OGL_Renderer::DrawImguiGraph();
}

void* DemoContextOGL::getGraphicsCommandQueue()
{
  return OGL_Renderer::GetGraphicsCommandQueue();
}

void DemoContextOGL::getRenderDevice(void** device, void** context)
{
  OGL_Renderer::GetRenderDevice(device, context);
}

void DemoContextOGL::bindHydrographicShader(Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ::ShadowMap* shadowMap, float bias, Vec4 fogColor)
{
  OGL_Renderer::BindHydrographicShader(lightPos, lightTarget, lightTransform, shadowMap, bias, fogColor);
}

void render()
{
  GLfloat vertices[] =
  {
    -0.5f, -0.5f, 0.0f,
    0.5f, -0.5f, 0.0f,
    0.0f,  0.5f, 0.0f
  };
  static const char *vertexShader =
    "#version 330 core\n"
    "layout(location = 0) in vec2 posAttr;\n"
    "void main() {\n"
    "gl_Position = vec4(posAttr, 0.0, 1.0); }";
  static const char *fragmentShader =
    "#version 330 core\n"
    "out vec4 col;\n"
    "void main() {\n"
    "col = vec4(1.0, 0.0, 0.0, 1.0); }";
  GLuint vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShaderID, 1, &vertexShader, nullptr);
  glCompileShader(vertexShaderID);
  GLuint fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShaderID, 1, &fragmentShader, nullptr);
  glCompileShader(fragmentShaderID);
  GLuint shaderProgramID = glCreateProgram();
  glAttachShader(shaderProgramID, vertexShaderID);
  glAttachShader(shaderProgramID, fragmentShaderID);
  glLinkProgram(shaderProgramID);
  glDetachShader(shaderProgramID, vertexShaderID);
  glDetachShader(shaderProgramID, fragmentShaderID);
  glDeleteShader(vertexShaderID);
  glDeleteShader(fragmentShaderID);
  GLuint vertexBufferID;
  glGenBuffers(1, &vertexBufferID);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
  glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(GLfloat), &vertices[0], GL_STATIC_DRAW);
  glUseProgram(shaderProgramID);
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glUseProgram(NULL);
  glDisableVertexAttribArray(0);
}

void StartFrameV2(Vec4 clearColor)
{
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);

  glPointSize(5.0f);
  glVerify(glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, g_msaaFbo));
  glVerify(glClearColor(SKY_COLOR.x, SKY_COLOR.y, SKY_COLOR.z, SKY_COLOR.w));
  glVerify(glClear(GL_COLOR_BUFFER_BIT));
}

void EndFrameV2()
{
  if (g_msaaFbo)
  {
    // blit the msaa buffer to the window
    glVerify(glBindFramebuffer(GL_READ_FRAMEBUFFER_EXT, g_msaaFbo));
    glVerify(glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, 0));
    glVerify(glBlitFramebuffer(0, 0, g_screenWidth, g_screenHeight, 0, 0, g_screenWidth, g_screenHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR));
  }
  // render help to back buffer
  glVerify(glBindFramebuffer(GL_FRAMEBUFFER, 0));
  glVerify(glClear(GL_DEPTH_BUFFER_BIT));
}

void drawPlane() {
  GLuint planeVBO = 0, planeVAO = 0;
  if (planeVBO == 0)
  {
    // vertices
    Vec3 pos1(-3.0f, 0.0f, 3.0f);
    Vec3 pos2(-3.0f, 0.0f, -3.0f);
    Vec3 pos3(3.0f, 0.0f, -3.0f);
    Vec3 pos4(3.0f, 0.0f, 3.0f);
    // cordenadas de textura
    Vec2 uv1(0.0f, 1.0f);
    Vec2 uv2(0.0f, 0.0f);
    Vec2 uv3(1.0f, 0.0f);
    Vec2 uv4(1.0f, 1.0f);
    // normal
    Vec3 nm(0.0f, 1.0f, 0.0f);
    float planeVertices[] = {
      // posicoes            // normal         // coordenadas de textura
      // triangulo 1
      pos1.x, pos1.y, pos1.z, nm.x, nm.y, nm.z, uv1.x, uv1.y,
      pos2.x, pos2.y, pos2.z, nm.x, nm.y, nm.z, uv2.x, uv2.y,
      pos3.x, pos3.y, pos3.z, nm.x, nm.y, nm.z, uv3.x, uv3.y,
      // triangulo 2
      pos1.x, pos1.y, pos1.z, nm.x, nm.y, nm.z, uv1.x, uv1.y,
      pos3.x, pos3.y, pos3.z, nm.x, nm.y, nm.z, uv3.x, uv3.y,
      pos4.x, pos4.y, pos4.z, nm.x, nm.y, nm.z, uv4.x, uv4.y,
    };
    // configure plane VAO
    glVerify(glGenVertexArrays(1, &planeVAO));
    glVerify(glGenBuffers(1, &planeVBO));
    glVerify(glBindVertexArray(planeVAO));
    glVerify(glBindBuffer(GL_ARRAY_BUFFER, planeVBO));
    glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW));
    glVerify(glEnableVertexAttribArray(0));
    glVerify(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0));
    glVerify(glEnableVertexAttribArray(1));
    glVerify(glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float))));
    glVerify(glEnableVertexAttribArray(2));
    glVerify(glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float))));
  }
  glVerify(glBindVertexArray(planeVAO));
  glVerify(glDrawArrays(GL_TRIANGLES, 0, 6));
  glVerify(glBindVertexArray(0));

}


void drawPlaneV2() {
  GLuint planeVBO = 0, planeVAO = 0, planeIBO = 0;
  GLuint nIndices;
  if (planeVBO == 0)
  {
    // vertices
    Vec3 pos1(-3.0f, 0.0f, 3.0f);
    Vec3 pos2(-3.0f, 0.0f, -3.0f);
    Vec3 pos3(3.0f, 0.0f, -3.0f);
    Vec3 pos4(3.0f, 0.0f, 3.0f);
    // cordenadas de textura
    Vec2 uv1(0.0f, 1.0f);
    Vec2 uv2(0.0f, 0.0f);
    Vec2 uv3(1.0f, 0.0f);
    Vec2 uv4(1.0f, 1.0f);
    // normal
    Vec3 nm(0.0f, 1.0f, 0.0f);
    //int nVertices = 6;
    int nVertices = 4;
    //
    float planeVertices[] = {
      // posicoes            
      // triangulo 1
      pos1.x, pos1.y, pos1.z,
      pos2.x, pos2.y, pos2.z,
      pos3.x, pos3.y, pos3.z,
      // triangulo 2
      //pos1.x, pos1.y, pos1.z,
      //pos3.x, pos3.y, pos3.z,
      pos4.x, pos4.y, pos4.z
    };
    // normal         
    float planeNormals[] = {
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f
    };
    // coordenadas de textura
    float planeTexCoords[] = {
      uv1.x, uv1.y,
      uv2.x, uv2.y,
      uv3.x, uv3.y,
      //uv1.x, uv1.y,
      //uv3.x, uv3.y,
      uv4.x, uv4.y
    };    
    //indices
    int indices[] = {
      0, 1, 2, 0, 2, 3
    };

    int sizeofFloat = sizeof(float);
    int sizeofVec3 = sizeof(Vec3);
    int sizeofVertices = sizeof(planeVertices);
    nIndices = 6;
    // configure plane VAO
    glVerify(glGenVertexArrays(1, &planeVAO));
    glVerify(glGenBuffers(1, &planeVBO));
    glVerify(glGenBuffers(1, &planeIBO));
    glVerify(glBindVertexArray(planeVAO));
    glVerify(glBindBuffer(GL_ARRAY_BUFFER, planeVBO));
    glVerify(glBufferData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec3) + sizeof(Vec3) + sizeof(Vec2)), NULL, GL_STATIC_DRAW));

    glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, nVertices * sizeof(Vec3), &planeVertices[0]));
    glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * sizeof(Vec3), nVertices * sizeof(Vec3), &planeNormals[0]));
    glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec3) + sizeof(Vec3)), nVertices * sizeof(Vec2), &planeTexCoords));

    glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeIBO));
    glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, nIndices * sizeof(int), &indices[0], GL_STATIC_DRAW));

    glVerify(glEnableVertexAttribArray(0));
    glVerify(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0));
    glVerify(glEnableVertexAttribArray(1));
    glVerify(glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)(nVertices * sizeof(Vec3))));
    glVerify(glEnableVertexAttribArray(2));
    glVerify(glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)(nVertices * (sizeof(Vec3) + sizeof(Vec3)))));

  }

  nIndices = 6;

  glVerify(glBindVertexArray(planeVAO));
  glVerify(glDrawElements(GL_TRIANGLES, nIndices, GL_UNSIGNED_INT, 0));
  glVerify(glBindVertexArray(0));

}

void ReshapeRenderV2(SDL_Window* window)
{
  int width, height;
  SDL_GetWindowSize(window, &width, &height);

  glViewport(0, 0, width, height);

  g_screenWidth = width;
  g_screenHeight = height;
}

void DrawGpuMeshV2(GpuMesh* m, const Matrix44& modelMat, bool showTexture)
{
  if (m)
  {
    int hasTexture = showTexture && m->texCoordsRigid.size() > 0 ? 1 : 0;
    if (hasTexture)
    {
      //Enable texture
      glVerify(glBindTexture(GL_TEXTURE_2D, m->mTextureId));
      glVerify(glEnable(GL_TEXTURE_2D));
      glVerify(glActiveTexture(GL_TEXTURE0));
      glVerify(glUniform1i(glGetUniformLocation(s_rigidBodyProgram, "tex"), 0));
      //Consider if it has a texture loaded to bind it
      glVerify(glUniform1i(glGetUniformLocation(s_rigidBodyProgram, "showTexture"), hasTexture));
    }
    glVerify(glUniform1i(glGetUniformLocation(s_rigidBodyProgram, "showPhong"), 1));
    //Setup normal and model mat
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_rigidBodyProgram, "normalMat"), 1, false, Transpose(AffineInverse(modelMat))));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_rigidBodyProgram, "model"), 1, false, modelMat));

    // bind buffers
    glBindBuffer(GL_ARRAY_BUFFER, m->mPositionsVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, m->mNormalsVBO);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    if (hasTexture)
    {
      glBindBuffer(GL_ARRAY_BUFFER, m->mTexCoordsVBO);
      glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
      glEnableVertexAttribArray(2);
    }

    glDrawArrays(GL_TRIANGLES, 0, m->mNumVertices);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    if (hasTexture)
    {
      // disable texture vertex attrib array
      glDisableVertexAttribArray(2);
      // disable texture
      glActiveTexture(GL_TEXTURE0);
      glDisable(GL_TEXTURE_2D);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  }
}

void SetupFilmMesh(GpuMesh* gpuMesh, GpuMesh* filmMesh)
{
  // update texture id
  filmMesh->mTextureId = gpuMesh->mTextureId;
}

// get texture coords from baricentric coordinates
Vec2 InterpolateTextureCoordinates(Vec2 textCoordV0, Vec2 textCoordV1, Vec2 textCoordV2, float u, float v, float w)
{
  Vec2 interpolatedTexCoords = w * textCoordV0 + u * textCoordV1 + v * textCoordV2;
  interpolatedTexCoords.x = Clamp(interpolatedTexCoords.x, 0.0f, 1.0f);
  interpolatedTexCoords.y = Clamp(interpolatedTexCoords.y, 0.0f, 1.0f);
  return interpolatedTexCoords;
}


void FindTextureSeam(Vec3 v0, Vec3 v1, Vec3 v2, Vec2 textCoordV0, Vec2 textCoordV1, Vec2 textCoordV2, PngImage textureImage)
{
  // parameters
  float epsilon = 0.00125f;
  float treshold = 0.15f;
  float wV0, uV1, vV2;
  float u, v, w;
  Vec2 textureAnt;
  // initial values

  // trasverse a triangle iterating barycentric coordinates and ploting the area inside a texture seam
  for (wV0 = 1.0f, uV1 = vV2 = 0.0f; wV0 >= .0f, vV2 <= 1.0f; wV0 -= epsilon, vV2 += epsilon)
  {
    for (u = .0f, v = vV2, w = wV0, textureAnt = wV0 * textCoordV0 + uV1 * textCoordV1 + vV2 * textCoordV2; u <= 1.0f, v <=1.0f, w >= .0f; u += epsilon, w -= epsilon, v = 1 - (u + w))
    {
      Vec3 p = w * v0 + u * v1 + v * v2; // compute arbitrary position given barycentric coordinates
      // Debug triangle sampler
      // com o triangulo da malha rigida, encontrar o ponto de interseccao e reinterpolar as coordenadas
      // detectar a descontinuidade na reinterpolacao
      Vec2 textureP = InterpolateTextureCoordinates(textCoordV0, textCoordV1, textCoordV2, u, v, w);
      float textureDistance = Length(textureP - textureAnt);

      PlotTexturePixel(p, textureP, textureImage);

      textureAnt = textureP;
    }
  }
}


void PlotTexturePixel(Vec3 position, Vec2 textureCoords, PngImage textureImage) 
{
  // get textel coordinates
  int textelCoordX = textureCoords.x * textureImage.m_width;
  int textelCoordY = textureCoords.y * textureImage.m_height;

  // avoid textel coordinates out of range
  textelCoordX = textelCoordX > textureImage.m_width - 1 ? textureImage.m_width - 1 : textelCoordX;
  textelCoordY = textelCoordY > textureImage.m_height - 1 ? textureImage.m_height - 1 : textelCoordY;

  textelCoordX = textelCoordX < 0 ? 0 : textelCoordX;
  textelCoordY = textelCoordY < 0 ? 0 : textelCoordY;

  // consider each pixel in rgba format by stb_load image procedure with 4 chanels
  GLuint texturePixel = textureImage.m_data[textelCoordX + textureImage.m_height * textelCoordY];

  unsigned char a = texturePixel >> 24 & 255;
  unsigned char b = texturePixel >> 16 & 255;
  unsigned char g = texturePixel >> 8 & 255;
  unsigned char r = texturePixel >> 0 & 255;

  Vec4 pixelTextureColor = Vec4((float)r / 255.0f, (float)g / 255.0f, float(b) / 255.0f, (float)a / 255.0f);
  
  // to a better performance consider begin point and end point procedure called outside this function
  // because this function will be called many times in a loop
  // begin points (called outside)
  DrawPoint(position, pixelTextureColor);
  // end points (called outside)

}

inline int GridIndex(int x, int y, int dx) { return y*dx + x; } // duplicated fnc

void BuildColorCompensation(Vec4* compensColors, Vec4* filmPositions, std::vector<Vec4> flatFilmPositions, const int dimX, const int dimZ)
{
  ColorGradient *colorGradient = new ColorGradient();

  for (int z = 0; z < dimZ; z++)
  {
    for (int x = 0; x < dimX; x++)
    {
      float restLength[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
      float finalLength[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
      int nearbyCount = 8;
      
      if (x > 0 && z > 0 && x < (dimX - 1) && z < (dimZ - 1))
      {
        restLength[0] = Length(Vec3(flatFilmPositions[GridIndex(x, z, dimX)]) - Vec3(flatFilmPositions[GridIndex(x, z - 1, dimX)]));
        restLength[1] = Length(Vec3(flatFilmPositions[GridIndex(x, z, dimX)]) - Vec3(flatFilmPositions[GridIndex(x + 1, z, dimX)]));
        restLength[2] = Length(Vec3(flatFilmPositions[GridIndex(x, z, dimX)]) - Vec3(flatFilmPositions[GridIndex(x, z + 1, dimX)]));
        restLength[3] = Length(Vec3(flatFilmPositions[GridIndex(x, z, dimX)]) - Vec3(flatFilmPositions[GridIndex(x - 1, z, dimX)]));

        restLength[4] = Length(Vec3(flatFilmPositions[GridIndex(x, z, dimX)]) - Vec3(flatFilmPositions[GridIndex(x - 1, z - 1, dimX)]));
        restLength[5] = Length(Vec3(flatFilmPositions[GridIndex(x, z, dimX)]) - Vec3(flatFilmPositions[GridIndex(x - 1, z + 1, dimX)]));
        restLength[6] = Length(Vec3(flatFilmPositions[GridIndex(x, z, dimX)]) - Vec3(flatFilmPositions[GridIndex(x + 1, z + 1, dimX)]));
        restLength[7] = Length(Vec3(flatFilmPositions[GridIndex(x, z, dimX)]) - Vec3(flatFilmPositions[GridIndex(x - 1, z + 1, dimX)]));

        finalLength[0] = Length(Vec3(filmPositions[GridIndex(x, z, dimX)]) - Vec3(filmPositions[GridIndex(x, z - 1, dimX)]));
        finalLength[1] = Length(Vec3(filmPositions[GridIndex(x, z, dimX)]) - Vec3(filmPositions[GridIndex(x + 1, z, dimX)]));
        finalLength[2] = Length(Vec3(filmPositions[GridIndex(x, z, dimX)]) - Vec3(filmPositions[GridIndex(x, z + 1, dimX)]));
        finalLength[3] = Length(Vec3(filmPositions[GridIndex(x, z, dimX)]) - Vec3(filmPositions[GridIndex(x - 1, z, dimX)]));

        finalLength[4] = Length(Vec3(filmPositions[GridIndex(x, z, dimX)]) - Vec3(filmPositions[GridIndex(x - 1, z - 1, dimX)]));
        finalLength[5] = Length(Vec3(filmPositions[GridIndex(x, z, dimX)]) - Vec3(filmPositions[GridIndex(x - 1, z + 1, dimX)]));
        finalLength[6] = Length(Vec3(filmPositions[GridIndex(x, z, dimX)]) - Vec3(filmPositions[GridIndex(x + 1, z + 1, dimX)]));
        finalLength[7] = Length(Vec3(filmPositions[GridIndex(x, z, dimX)]) - Vec3(filmPositions[GridIndex(x + 1, z - 1, dimX)]));
      }
      else
      {
        finalLength[0] = restLength[0] = 1.0f;
      }

      float variationAcum = 0.0f;

      for (int count = 0; count < nearbyCount; count++)
      {
        variationAcum += abs(finalLength[count] - restLength[count]) / restLength[count];
      }

      float avgVariation = variationAcum / nearbyCount;

      compensColors[GridIndex(x, z, dimX)] = avgVariation;//Vec4(Vec3(colorGradient->getColorAtValue(avgVariation)), 1.0f);

    }
  }
}


void BuildContactUVs(Vec3 filmContactVertex, int filmContactVertexIndex, Vec3 filmContactPlane, GpuMesh* gpuMesh, Mat44 modelMatrix, std::vector<Vec4> contactPositions, std::vector<Vec4> &contactUVs)
{
  // look for every triangle in the rigid body to cast a ray
  for (int i = 0; i < gpuMesh->triangles.size(); ++i)
  {
    // process only vertices without texture coordinates
    if (contactUVs[filmContactVertexIndex].x == -1.0f)
    {
      Vec3 v0 = Vec3(modelMatrix * Vec4(gpuMesh->positions[i * 3 + 0], 1.0f));
      Vec3 v1 = Vec3(modelMatrix * Vec4(gpuMesh->positions[i * 3 + 1], 1.0f));
      Vec3 v2 = Vec3(modelMatrix * Vec4(gpuMesh->positions[i * 3 + 2], 1.0f));

      float t = INFINITY;
      float u, v, w;
      // apply Möller–Trumbore algorithm to cast a ray from the contact point to the triangle
      if (gpuMesh->texCoordsRigid.size() && rayTriangleIntersectMT(filmContactVertex, filmContactPlane, v0, v1, v2, t, u, v, w))
      {
        // interpolate texture coordinates by barycentric coordinates and triangle texture coordinates
        Vec2 texCoordsRigid = InterpolateTextureCoordinates(gpuMesh->texCoordsRigid[i * 3 + 0],
          gpuMesh->texCoordsRigid[i * 3 + 1], gpuMesh->texCoordsRigid[i * 3 + 2], u, v, w);

        // update the texture coordinates to be used in the flat film
        contactUVs[filmContactVertexIndex] = Vec4(texCoordsRigid.x, texCoordsRigid.y, 0.0f, 0.0f);

      }

    }

  }

}


void DetectTextureSeams(GpuMesh* filmMesh, std::vector<Vec4> &contactPositions, std::vector<Vec4> &contactUVs)
{
  float maxDistanceUV = .3f;
  Vec4 red = Vec4(1.0f, 0.0f, 0.0f, 1.0f);
  Vec4 blu = Vec4(0.0f, 0.0f, 1.0f, 1.0f);
  Vec4 gre = Vec4(0.0f, 1.0f, 0.0f, 1.0f);

  // for all triangle indexes in triangle film mesh
  for (int i = 0; i < filmMesh->triangleIntIndexes.size(); i = i + 3) //
  {
    // for a triangle
    int index0 = filmMesh->triangleIntIndexes[i];
    int index1 = filmMesh->triangleIntIndexes[i + 1];
    int index2 = filmMesh->triangleIntIndexes[i + 2];

    Vec3 texCoord0 = Vec3(contactUVs[index0]);
    Vec3 texCoord1 = Vec3(contactUVs[index1]);
    Vec3 texCoord2 = Vec3(contactUVs[index2]);
   
    // if texture coordinat is set
    if (texCoord0.x != -1 && texCoord1.x != -1 && texCoord2.x != -1)
    {
      float distance01 = Length(texCoord0 - texCoord1);
      float distance02 = Length(texCoord0 - texCoord2);
      float distance12 = Length(texCoord1 - texCoord2);
      // texture distance in a triangle edge is greather than treshold maxDistanceUV
      // is a candidate texture seam
      if (distance01 > maxDistanceUV || distance02 > maxDistanceUV || distance12 > maxDistanceUV)
      { 

        Vec3 half01 = .5f * (filmMesh->positions[index0] + filmMesh->positions[index1]);
        Vec3 half02 = .5f * (filmMesh->positions[index0] + filmMesh->positions[index2]);
        Vec3 half12 = .5f * (filmMesh->positions[index1] + filmMesh->positions[index2]);

        // detect what edges of the triangle cross the seam 
        // v0 is in a vertex that cross the seam

        if (distance01 > maxDistanceUV && distance02 > maxDistanceUV)
        {
          BeginLines();
          DrawLine(filmMesh->positions[index0], filmMesh->positions[index1], blu);
          DrawLine(filmMesh->positions[index0], filmMesh->positions[index2], blu);
          EndLines();

          BeginPoints(4.0);
          DrawPoint(filmMesh->positions[index0], red);
          DrawPoint(half01, gre);
          DrawPoint(half02, gre);
          EndPoints();
          //DrawPoint(filmMesh->positions[index1], blu);
          //DrawPoint(filmMesh->positions[index2], blu);
        }
        // v1 is in a vertex that cross the seam
        if (distance01 > maxDistanceUV && distance12 > maxDistanceUV)
        {
          BeginLines();
          DrawLine(filmMesh->positions[index0], filmMesh->positions[index1], blu);
          DrawLine(filmMesh->positions[index1], filmMesh->positions[index2], blu);
          EndLines();

          //DrawPoint(filmMesh->positions[index0], blu);
          BeginPoints(4.0);
          DrawPoint(filmMesh->positions[index1], red);
          DrawPoint(half01, gre);
          DrawPoint(half12, gre);
          EndPoints();
          //DrawPoint(filmMesh->positions[index2], blu);
        }
        // v2 is in a vertex that cross the seam
        if (distance02 > maxDistanceUV && distance12 > maxDistanceUV)
        {
          BeginLines();
          DrawLine(filmMesh->positions[index0], filmMesh->positions[index2], blu);
          DrawLine(filmMesh->positions[index1], filmMesh->positions[index2], blu);
          EndLines();

          //DrawPoint(filmMesh->positions[index0], blu);
          //DrawPoint(filmMesh->positions[index1], blu);
          BeginPoints(4.0);
          DrawPoint(filmMesh->positions[index2], red);
          DrawPoint(half02, gre);
          DrawPoint(half12, gre);
          EndPoints();
        }
              
      }
      
    }    
  }

}


void ReadDisplacements(int* backbuffer, int width, int height)
{
  glVerify(glReadBuffer(GL_BACK));
  glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, backbuffer);
}

void WriteDisplacements(int* pixels, int width, int height)
{
  // if CHANNEL_NUM is 4, you can use alpha channel in png
#define CHANNEL_NUM 4

  stbi_write_png("../../movies/stbpng.png", width, height, CHANNEL_NUM, pixels, width * CHANNEL_NUM);
}

size_t traverseScene(const aiScene *scene, const aiNode* node, Mesh* mesh, std::string basePath) {
  size_t nVertices = 0;
  for (size_t i = 0; i < node->mNumMeshes; i++) {
    const aiMesh* assimp_mesh = scene->mMeshes[node->mMeshes[i]];
    // material properties
    const aiMaterial* const mtl_properties = scene->mMaterials[assimp_mesh->mMaterialIndex];
    aiColor4D Ka, Kd, Ks;
    aiString map_Kd;
    std::string textureFile;

    if (AI_SUCCESS == mtl_properties->Get(AI_MATKEY_COLOR_AMBIENT, Ka)) 
    {
      mesh->Ka = Vec4(Ka.r, Ka.g, Ka.b, Ka.a);
    }
    if (AI_SUCCESS == mtl_properties->Get(AI_MATKEY_COLOR_DIFFUSE, Kd)) 
    {
      mesh->Kd = Vec4(Kd.r, Kd.g, Kd.b, Kd.a);
    }
    if (AI_SUCCESS == mtl_properties->Get(AI_MATKEY_COLOR_SPECULAR, Ks)) 
    {
      mesh->Ks = Vec4(Ks.r, Ks.g, Ks.b, Ks.a);
    }
    if (AI_SUCCESS == mtl_properties->Get(AI_MATKEY_TEXTURE_DIFFUSE(i), map_Kd)) 
    {
      std::string texturePath = basePath + '/' + map_Kd.C_Str();
      mesh->mTextureId = LoadTexture(texturePath.c_str(), g_rigid_model_texture_image);
      if (mesh->mTextureId == 0)
      {
        std::cout << "Texture loading error" << std::endl;
      }
    }
    else 
    {
      mesh->mTextureId = 0;
      std::cout << "No texture loaded" << std::endl;
    }

    for (size_t j = 0; j < assimp_mesh->mNumFaces; j++) {
      const aiFace* face = &assimp_mesh->mFaces[j];

      Triangle triangle;

      for (size_t k = 0; k < face->mNumIndices; k++) {
        int index = face->mIndices[k];

        if (k < 3)
        {
          triangle.verticeIndexes[k] = face->mIndices[k];
        }

        if (assimp_mesh->HasVertexColors(0)) // not tested yet
        {
          mesh->m_colours.push_back(Colour(assimp_mesh->mColors[0]->r, assimp_mesh->mColors[0]->g, assimp_mesh->mColors[0]->b, assimp_mesh->mColors[0]->a));
        }
        else 
        {
          mesh->m_colours.push_back(Colour(mesh->Ka));
        }
        if (assimp_mesh->HasTextureCoords(0) && mesh->mTextureId) // load texture coords if has mTextureId
        {
          mesh->m_texcoords[0].push_back(Vec2(assimp_mesh->mTextureCoords[0][index][0], assimp_mesh->mTextureCoords[0][index][1]));
        }
        if (assimp_mesh->HasNormals()) 
        {
          mesh->m_normals.push_back(Vec3(assimp_mesh->mNormals[index].x, assimp_mesh->mNormals[index].y, assimp_mesh->mNormals[index].z));
        }
        mesh->m_positions.push_back(Point3(assimp_mesh->mVertices[index].x, assimp_mesh->mVertices[index].y, assimp_mesh->mVertices[index].z));
        TriangleIndexes triangleIndexes;

        mesh->m_triangle_index.push_back(triangleIndexes);
        nVertices++;
      }
      mesh->m_triangles.push_back(triangle);
    }
  }

  for (size_t i = 0; i < node->mNumChildren; i++)
  {
    nVertices += traverseScene(scene, node->mChildren[i], mesh, basePath);
  }

  return nVertices;
}

void createVBOs(const aiScene *scene, GpuMesh* gpu_mesh, std::string basePath, Mat44 transformation, float margin) {

  std::cout << "Scene:     #Meshes     = " << scene->mNumMeshes << std::endl;
  std::cout << "           #Textures   = " << scene->mNumTextures << std::endl;

  Mesh* mesh = new Mesh();
  gpu_mesh->mNumVertices = GLuint(traverseScene(scene, scene->mRootNode, mesh, basePath));
  g_rigid_model_texture_id = gpu_mesh->mTextureId = GLuint(mesh->mTextureId); // share rigid model texture id with the reverse mapping film 
  gpu_mesh->triangles = mesh->m_triangles;

  std::cout << "           #Vertices   = " << gpu_mesh->mNumVertices << std::endl;
  std::cout << "           #vboVertices= " << mesh->m_positions.size() << std::endl;
  std::cout << "           #vboColors= " << mesh->m_colours.size() << std::endl;
  std::cout << "           #vboNormals= " << mesh->m_normals.size() << std::endl;
  std::cout << "           #vboTxCoords= " << mesh->m_texcoords[0].size() << std::endl;

  // same transformations when create sdf mesh
  mesh->Transform(transformation);
  mesh->Normalize(1.0f - margin);
  mesh->Transform(TranslationMatrix(Point3(margin, margin, margin)*0.5f));

  float maxDistanceUV = 0.0f;
  float minDistanceUV = 1.0f;
  float avgDistanceUV = 0.0f;
  int countDistanceUV = 0;
  float sumDistanceUV = 0.0f;

  for (int i = 0; i < mesh->m_positions.size(); i++)
  {
    // can't copy directly because of type incompatibility between point and Vec3
    gpu_mesh->positions.push_back((Vec3)mesh->m_positions[i]);
    gpu_mesh->normals.push_back(mesh->m_normals[i]);

    // and also because m_texcoords is an array with dimensions [0][n];
    if (mesh->m_texcoords[0].size() == mesh->m_positions.size())
    {
      gpu_mesh->texCoordsRigid.push_back(mesh->m_texcoords[0][i]);
    }

    if (i % 3 == 0)
    {
      float texDistance01 = Length(mesh->m_positions[i] - mesh->m_positions[i+1]);
      float texDistance12 = Length(mesh->m_positions[i+1] - mesh->m_positions[i+2]);
      float texDistance02 = Length(mesh->m_positions[i] - mesh->m_positions[i+2]);

      maxDistanceUV = Max(maxDistanceUV, Max(texDistance01, Max(texDistance12, texDistance02)));
      minDistanceUV = Min(minDistanceUV, Min(texDistance01, Min(texDistance12, texDistance02)));

      sumDistanceUV = sumDistanceUV + (texDistance01 + texDistance12 + texDistance02);
      countDistanceUV += 3;
    }

    avgDistanceUV = sumDistanceUV / countDistanceUV;

  }

  glGenBuffers(1, &gpu_mesh->mPositionsVBO);
  glBindBuffer(GL_ARRAY_BUFFER, gpu_mesh->mPositionsVBO);
  glBufferData(GL_ARRAY_BUFFER, mesh->m_positions.size() * sizeof(Vec3), mesh->m_positions.data(), GL_STATIC_DRAW);

  if (gpu_mesh->colors.size() > 0)
  {
    glGenBuffers(1, &gpu_mesh->mColorsVBO);
    glBindBuffer(GL_ARRAY_BUFFER, gpu_mesh->mColorsVBO);
    glBufferData(GL_ARRAY_BUFFER, mesh->m_colours.size() * sizeof(Vec4), mesh->m_colours.data(), GL_STATIC_DRAW);
  }
  if (mesh->m_normals.size() > 0)
  {
    glGenBuffers(1, &gpu_mesh->mNormalsVBO);
    glBindBuffer(GL_ARRAY_BUFFER, gpu_mesh->mNormalsVBO);
    glBufferData(GL_ARRAY_BUFFER, mesh->m_normals.size() * sizeof(Vec3), mesh->m_normals.data(), GL_STATIC_DRAW);
  }
  if (mesh->m_texcoords[0].size() > 0) {
    glGenBuffers(1, &gpu_mesh->mTexCoordsVBO);
    glBindBuffer(GL_ARRAY_BUFFER, gpu_mesh->mTexCoordsVBO);
    glBufferData(GL_ARRAY_BUFFER, mesh->m_texcoords[0].size() * sizeof(Vec2), mesh->m_texcoords[0].data(), GL_STATIC_DRAW);
  }
  std::cout << "           #meshSize= " << gpu_mesh->mNumVertices << std::endl;

  delete mesh;

}

void get_bounding_box_for_node(const struct aiNode* nd, aiVector3D* min, aiVector3D* max, aiMatrix4x4* trafo) {
  aiMatrix4x4 prev;
  unsigned int n = 0, t;

  prev = *trafo;
  aiMultiplyMatrix4(trafo, &nd->mTransformation);

  for (; n < nd->mNumMeshes; ++n) {
    const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
    for (t = 0; t < mesh->mNumVertices; ++t) {
      aiVector3D tmp = mesh->mVertices[t];
      aiTransformVecByMatrix4(&tmp, trafo);

      min->x = MIN(min->x, tmp.x);
      min->y = MIN(min->y, tmp.y);
      min->z = MIN(min->z, tmp.z);

      max->x = MAX(max->x, tmp.x);
      max->y = MAX(max->y, tmp.y);
      max->z = MAX(max->z, tmp.z);
    }
  }

  for (n = 0; n < nd->mNumChildren; ++n) {
    get_bounding_box_for_node(nd->mChildren[n], min, max, trafo);
  }
  *trafo = prev;
}

GpuMesh* CreateGpuMesh(const char* filename, Mat44 transformation, float margin) {

  GpuMesh* gpu_mesh = new GpuMesh();
  aiPropertyStore* props = aiCreatePropertyStore(); aiSetImportPropertyInteger(props, "PP_PTV_NORMALIZE", 1);
  scene = (aiScene*)aiImportFileExWithProperties(filename, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_PreTransformVertices, NULL, props);
  aiReleasePropertyStore(props);
  //scene = aiImportFile(filename, aiProcessPreset_TargetRealtime_MaxQuality);
  if (!scene) {
    std::cout << "## ERROR loading mesh" << std::endl;
    exit(-1);
  }

  aiMatrix4x4 trafo;
  aiIdentityMatrix4(&trafo);

  aiVector3D min, max;

  scene_min.x = scene_min.y = scene_min.z = FLT_MAX;
  scene_max.x = scene_max.y = scene_max.z = -FLT_MAX;

  get_bounding_box_for_node(scene->mRootNode, &scene_min, &scene_max, &trafo);

  scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
  scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
  scene_center.z = (scene_min.z + scene_max.z) / 2.0f;

  scene_min.x *= ai_real(1.2);
  scene_min.y *= ai_real(1.2);
  scene_min.z *= ai_real(1.2);
  scene_max.x *= ai_real(1.2);
  scene_max.y *= ai_real(1.2);
  scene_max.z *= ai_real(1.2);

  if (scene_list == 0) 
  {
    std::string p = std::string(filename);
    std::size_t found = p.find_last_of('/');
    std::string basePath = p.substr(0, found);
    createVBOs(scene, gpu_mesh, basePath, transformation, margin);
  }

  std::cout << "Bounding Box: " << " (" << scene_min.x << " , " << scene_min.y << " , " << scene_min.z << ") - (" << scene_max.x << " , " << scene_max.y << " , " << scene_max.z << ")" << std::endl;
  std::cout << "Bounding Box: " << " (" << scene_center.x << " , " << scene_center.y << " , " << scene_center.z << ")" << std::endl;
  
  gpu_mesh->center.x = scene_center.x;
  gpu_mesh->center.y = scene_center.y;
  gpu_mesh->center.z = scene_center.z;

  return gpu_mesh;
}

/*
void SetGpuMeshTriangles(GpuMesh* gpuMesh, std::vector<Triangle> triangles, std::vector<TriangleIndexes> triangleIndexes) {
  gpuMesh->triangles = triangles;
  gpuMesh->triangleIndexes = triangleIndexes;
}
*/

#define CULLING
/*
  Möller - Trumbore algorithm
  https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
*/
bool rayTriangleIntersectMT(Vec3 orig, Vec3 dir, Vec3 v0, Vec3 v1, Vec3 v2, float &t, float &u, float &v, float &w)
{
  float kEpsilon = 1e-6;//1e-8;
  Vec3 v0v1 = v1 - v0;
  Vec3 v0v2 = v2 - v0;
  Vec3 pvec = Cross(dir, v0v2);
  float det = Dot(v0v1, pvec);
#ifdef CULLING 
  // if the determinant is negative the triangle is backfacing
  // if the determinant is close to 0, the ray misses the triangle
  if (det < kEpsilon) return false;
#else 
  // ray and triangle are parallel if det is close to 0
  if (fabs(det) < kEpsilon) return false;
#endif 
  float invDet = 1 / det;

  Vec3 tvec = orig - v0;
  u = Dot(tvec, pvec) * invDet;
  if (u < 0 || u > 1) return false;

  Vec3 qvec = Cross(tvec, v0v1);
  v = Dot(dir, qvec) * invDet;
  if (v < 0 || u + v > 1) return false;

  w = 1 - u - v;
  t = Dot(v0v2, qvec) * invDet;

  return true;
}

GpuMesh* CreateGpuFilm(Matrix44 model, Vec4* positions, Vec4* normals, Vec4* uvs, int nVertices, int* indices, int nIndices)
{
  // buffer layout block
  // |all position vertices|all normal vertices|all tex coords|
  GpuMesh* mesh = new GpuMesh();
  mesh->mNumVertices = nVertices;
  mesh->mNumFaces = nIndices / 3;
  mesh->mNumIndices = nIndices;
  mesh->modelTransform = model;
  mesh->mTextureId = GetChessboardTextureId(); //set texture ID for film
  mesh->texCoordsFilm.resize(0);
  for (int i = 0; i < nVertices; i++)
  {
    mesh->positions.push_back(Vec3(positions[i]));
    mesh->texCoordsFilm.push_back(uvs[i]);
    mesh->colors.push_back(Vec4(1.0f));
  }

  for (int i = 0; i < nIndices; i++)
  {
    mesh->triangleIntIndexes.push_back(indices[i]);
  }

  // seam indexes buffer storage
  //glVerify(glGenBuffers(1, &mesh->mSeamIndexes));


  // configure plane VAO
  glVerify(glGenVertexArrays(1, &mesh->mVAO));
  glVerify(glGenBuffers(1, &mesh->mPositionsVBO));
  glVerify(glGenBuffers(1, &mesh->mIndicesIBO));

  glVerify(glBindVertexArray(mesh->mVAO));
  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO));
  //glVerify(glBufferData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec4) + sizeof(Vec4) + sizeof(Vec4)), NULL, GL_DYNAMIC_DRAW));
  glVerify(glBufferData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec4) + sizeof(Vec4) + sizeof(Vec4) + sizeof(Vec4)), NULL, GL_DYNAMIC_DRAW));

  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, nVertices * sizeof(Vec4), &positions[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * sizeof(Vec4), nVertices * sizeof(Vec4), &normals[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec4) + sizeof(Vec4)), nVertices * sizeof(Vec4), &uvs[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec4) + sizeof(Vec4) + sizeof(Vec4)), nVertices * sizeof(Vec4), &mesh->colors[0]));

  glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->mIndicesIBO));
  glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, nIndices * sizeof(int), &indices[0], GL_STATIC_DRAW));

  glVerify(glEnableVertexAttribArray(0));
  glVerify(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0));
  glVerify(glEnableVertexAttribArray(1));
  glVerify(glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(nVertices * sizeof(Vec4))));
  glVerify(glEnableVertexAttribArray(2));
  glVerify(glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(nVertices * (sizeof(Vec4) + sizeof(Vec4)))));
  glVerify(glEnableVertexAttribArray(3));
  glVerify(glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(nVertices * (sizeof(Vec4) + sizeof(Vec4) + sizeof(Vec4)))));

  glVerify(glBindVertexArray(0));

  return mesh;
}


// Create a NULL-terminated string by reading the provided file
static char* readShaderSource(const char* shaderFile) {

  FILE* fp = fopen(shaderFile, "r");

  if (fp == NULL) { return NULL; }

  fseek(fp, 0L, SEEK_END);
  long size = ftell(fp);

  fseek(fp, 0L, SEEK_SET);
  //char* buf = new char[size + 2];
  char* buf = new char[size];
  memset(buf, 0, size);
  fread(buf, 1, size, fp);

  //buf[size] = '\r';
  //buf[size + 1] = '\n';
  fclose(fp);

  return buf;
}


//https://stackoverflow.com/questions/7344640/getting-garbage-chars-when-reading-glsl-files
std::string loadFileToString(char const * const fname)
{
  std::ifstream ifile(fname);
  std::string filetext;

  while (ifile.good()) {
    std::string line;
    std::getline(ifile, line);
    filetext.append(line + "\n");
  }

  return filetext;
}


// Create a GLSL program object from vertex and fragment shader files

unsigned int InitShader(const Shader *shaders, const bool readFile) 
{
  GLuint program = glCreateProgram();

  // for each shader of the pipeline
  for (int i = 0; i < 5; ++i)
  {
    std::string shaderSource;
    Shader s = shaders[i];
    if (s.filename == NULL)
    {
      continue; // bypass optional shaders
    }

    if (readFile)
    {
      shaderSource = loadFileToString(s.filename);
    } 
    else
    {
      shaderSource = shaders[i].source;
    }

    std::cout << "Creating shader " << s.filename << std::endl;

    if (shaderSource.length() == 0)
    {
      std::cerr << "Failed to read " << s.filename << std::endl;
      exit(EXIT_FAILURE);
    }

    GLuint shader = glCreateShader(s.type);

    const GLchar *source = NULL;
    source = (const GLchar *)shaderSource.c_str();
    glShaderSource(shader, 1, &source, 0);
    glCompileShader(shader);

    GLint  compiled;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
      std::cerr << s.filename << " failed to compile:" << std::endl;
      GLint  logSize;
      glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logSize);
      char* logMsg = new char[logSize];
      glGetShaderInfoLog(shader, logSize, NULL, logMsg);
      std::cerr << logMsg << std::endl;
      delete[] logMsg;

      exit(EXIT_FAILURE);
    }

    std::cout << "Shader program " << s.filename << " compiled sucessfull " << std::endl;

    glAttachShader(program, shader);
  }


  /* link  and error check */
  glLinkProgram(program);

  GLint  linked;
  glGetProgramiv(program, GL_LINK_STATUS, &linked);
  if (!linked)
  {
    std::cerr << "Shader program failed to link" << std::endl;
    GLint  logSize;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logSize);
    char* logMsg = new char[logSize];
    glGetProgramInfoLog(program, logSize, NULL, logMsg);
    std::cerr << logMsg << std::endl;
    delete[] logMsg;

    exit(EXIT_FAILURE);
  }

  return program;
}

void _check_gl_error(const char *file, int line) {
  GLenum err(glGetError());

  while (err != GL_NO_ERROR) {
    std::string error;

    switch (err) {
    case GL_INVALID_OPERATION:      error = "INVALID_OPERATION";      break;
    case GL_INVALID_ENUM:           error = "INVALID_ENUM";           break;
    case GL_INVALID_VALUE:          error = "INVALID_VALUE";          break;
    case GL_OUT_OF_MEMORY:          error = "OUT_OF_MEMORY";          break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:  error = "INVALID_FRAMEBUFFER_OPERATION";  break;
    }

    std::cerr << "GL_" << error.c_str() << " - " << file << ":" << line << std::endl;
    err = glGetError();
  }
}

GLuint GetChessboardTextureId()
{
  return g_chessboard_texture_id;
}

GLuint GetRigidModelTextureId()
{
  return g_rigid_model_texture_id;
}

GLuint GetHeatmapTextureId()
{
  return g_heatmap_texture_id;
}

void SetViewport(int x, int y, int width, int height)
{
  glVerify(glViewport(x, y, width, height));
}

void GetFrustum(float l, float r, float b, float t, float n, float f)
{
  glFrustum(l, r, b, t, n, f);
}

void BindRigidBodyShader(Matrix44 view, Matrix44 proj, Vec3 lightPos, Vec3 camPos, Vec4 lightColor, Vec4 ambientColor, Vec4 specularColor, unsigned int specularExpoent, Vec4 diffuseColor)
{
  
  if (s_rigidBodyProgram)
  {
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    glVerify(glUseProgram(s_rigidBodyProgram));
    glVerify(glUniform3fv(glGetUniformLocation(s_rigidBodyProgram, "uLPos"), 1, lightPos));
    glVerify(glUniform4fv(glGetUniformLocation(s_rigidBodyProgram, "uLColor"), 1, lightColor));
    glVerify(glUniform4fv(glGetUniformLocation(s_rigidBodyProgram, "uColor"), 1, diffuseColor));
    glVerify(glUniform3fv(glGetUniformLocation(s_rigidBodyProgram, "uCamPos"), 1, camPos));
    glVerify(glUniform4fv(glGetUniformLocation(s_rigidBodyProgram, "uAmbient"), 1, ambientColor));
    glVerify(glUniform4fv(glGetUniformLocation(s_rigidBodyProgram, "uSpecular"), 1, specularColor));
    glVerify(glUniform1ui(glGetUniformLocation(s_rigidBodyProgram, "uSpecularExpoent"), specularExpoent));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_rigidBodyProgram, "view"), 1, false, &view[0]));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_rigidBodyProgram, "proj"), 1, false, &proj[0]));
  }
}

void BindFilmShader(Matrix44 view, Matrix44 proj, Vec3 lightPos, Vec3 camPos, Vec4 lightColor, Vec4 ambientColor, Vec4 specularColor, unsigned int specularExpoent, Vec4 diffuseColor)
{

  if (s_filmProgram)
  {
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    glVerify(glUseProgram(s_filmProgram));
    glVerify(glUniform3fv(glGetUniformLocation(s_filmProgram, "uLPos"), 1, lightPos));
    glVerify(glUniform4fv(glGetUniformLocation(s_filmProgram, "uLColor"), 1, lightColor));
    glVerify(glUniform4fv(glGetUniformLocation(s_filmProgram, "uColor"), 1, diffuseColor));
    glVerify(glUniform3fv(glGetUniformLocation(s_filmProgram, "uCamPos"), 1, camPos));
    glVerify(glUniform4fv(glGetUniformLocation(s_filmProgram, "uAmbient"), 1, ambientColor));
    glVerify(glUniform4fv(glGetUniformLocation(s_filmProgram, "uSpecular"), 1, specularColor));
    glVerify(glUniform1ui(glGetUniformLocation(s_filmProgram, "uSpecularExpoent"), specularExpoent));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_filmProgram, "view"), 1, false, &view[0]));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_filmProgram, "proj"), 1, false, &proj[0]));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_filmProgram, "normalMat"), 1, false, Transpose(AffineInverse(Matrix44::kIdentity))));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_filmProgram, "model"), 1, false, Matrix44::kIdentity));
  }
}

void SetReverseTextureParams()
{
  //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glActiveTexture(GL_TEXTURE0);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
  glActiveTexture(GL_TEXTURE1);
  glDisable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE2);
  glDisable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE3);
  glDisable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE4);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_CUBE_MAP);
  glActiveTexture(GL_TEXTURE5);
  glDisable(GL_TEXTURE_2D);

  glActiveTexture(GL_TEXTURE0);

  glDisable(GL_BLEND);
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glDisable(GL_POINT_SPRITE);

  // save scene camera transform
  //glMatrixMode(GL_MODELVIEW);
  //glPushMatrix();
  //glLoadIdentity();

  //const Matrix44 ortho = OrthographicMatrix(0.0f, float(g_screenWidth), 0.0f, float(g_screenHeight), -1.0f, 1.0f);

  //glMatrixMode(GL_PROJECTION);
  //glPushMatrix();
  //glLoadMatrixf(ortho);

  //glUseProgram(0);
  //glDisable(GL_DEPTH_TEST);
  //glDisable(GL_CULL_FACE);
  //glEnable(GL_BLEND);
  //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //glDisable(GL_TEXTURE_2D);
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

void UnsetReverseTextureParams()
{
  // restore camera transform (for picking)
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

void BindReverseTextureShader(Matrix44 view, Matrix44 proj, Vec3 lightPos, Vec3 camPos, 
  Vec4 lightColor, Vec4 ambientColor, Vec4 specularColor, unsigned int specularExpoent, 
  Vec4 diffuseColor, float maxDistanceUV, float nearDistanceUV, 
  float weight1, float weight2, float tesselationInner, float tesselationOuter)
{

  if (s_reverseTexProgram)
  {
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    glVerify(glUseProgram(s_reverseTexProgram));
    glVerify(glUniform1f(glGetUniformLocation(s_reverseTexProgram, "uTesselationInner"), tesselationInner));
    glVerify(glUniform1f(glGetUniformLocation(s_reverseTexProgram, "uTesselationOuter"), tesselationOuter));
    glVerify(glUniform1f(glGetUniformLocation(s_reverseTexProgram, "uMaxDistanceUV"), maxDistanceUV));
    glVerify(glUniform1f(glGetUniformLocation(s_reverseTexProgram, "uNearDistanceUV"), nearDistanceUV));
    glVerify(glUniform1f(glGetUniformLocation(s_reverseTexProgram, "uWeight1"), weight1));
    glVerify(glUniform1f(glGetUniformLocation(s_reverseTexProgram, "uWeight2"), weight2));
    glVerify(glUniform3fv(glGetUniformLocation(s_reverseTexProgram, "uLPos"), 1, lightPos));
    glVerify(glUniform4fv(glGetUniformLocation(s_reverseTexProgram, "uLColor"), 1, lightColor));
    glVerify(glUniform4fv(glGetUniformLocation(s_reverseTexProgram, "uColor"), 1, diffuseColor));
    glVerify(glUniform3fv(glGetUniformLocation(s_reverseTexProgram, "uCamPos"), 1, camPos));
    //glVerify(glUniform4fv(glGetUniformLocation(s_reverseTexProgram, "uAmbient"), 1, ambientColor));
    //glVerify(glUniform4fv(glGetUniformLocation(s_reverseTexProgram, "uSpecular"), 1, specularColor));
    //glVerify(glUniform1ui(glGetUniformLocation(s_reverseTexProgram, "uSpecularExpoent"), specularExpoent));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_reverseTexProgram, "view"), 1, false, &view[0]));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_reverseTexProgram, "proj"), 1, false, &proj[0]));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_reverseTexProgram, "normalMat"), 1, false, Transpose(AffineInverse(Matrix44::kIdentity))));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_reverseTexProgram, "model"), 1, false, Matrix44::kIdentity));
  }
}


void DrawHydrographicFilm(GpuMesh* mesh, const Vec4* positions, const Vec4* normals, const Vec4* uvs, const int* indices, int nIndices, int numPositions, bool showTexture)
{
  // Enable texture
  glVerify(glEnable(GL_TEXTURE_2D));//
  glVerify(glActiveTexture(GL_TEXTURE0));//
  glVerify(glBindTexture(GL_TEXTURE_2D, GetChessboardTextureId()));
  glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "tex"), 0));//
                                                                       //int hasTexture = showTexture && mesh->texCoordsFilm.size() > 0 ? 1 : 0;
                                                                       //glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "showTexture"), hasTexture));
  glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "showTexture"), showTexture));
  glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "showPhong"), 1));
  // update positions and normals
  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO)); // 
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, mesh->mNumVertices * sizeof(Vec4), positions));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(Vec4), mesh->mNumVertices * sizeof(Vec4), normals));
  if (showTexture)
  {
    glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * (sizeof(Vec4) + sizeof(Vec4)), mesh->mNumVertices * sizeof(Vec4), uvs));
  }
  // draw VAO
  glVerify(glBindVertexArray(mesh->mVAO));
  glVerify(glDrawElements(GL_TRIANGLES, nIndices, GL_UNSIGNED_INT, 0));
  glVerify(glBindVertexArray(0));
  // disable texture
  glActiveTexture(GL_TEXTURE0);
  glDisable(GL_TEXTURE_2D);
}

void DrawReverseTexture(GpuMesh* mesh, const Vec4* positions, const Vec4* normals, const Vec4* uvs, const int* indices, int nIndices, int numPositions, Vec4* stretchColors, int textureMode)
{
  glVerify(glUniform1i(glGetUniformLocation(s_reverseTexProgram, "textureMode"), textureMode));
  if (textureMode > 0)
  {
    // Enable texture
    glVerify(glEnable(GL_TEXTURE_2D));//
    glVerify(glActiveTexture(GL_TEXTURE0));//
    glVerify(glBindTexture(GL_TEXTURE_2D, GetRigidModelTextureId()));
    glVerify(glUniform1i(glGetUniformLocation(s_reverseTexProgram, "reverseTexture"), 0));//
  }
  if (textureMode > 1)
  {
    glVerify(glEnable(GL_TEXTURE_1D));
    glVerify(glActiveTexture(GL_TEXTURE1));
    glVerify(glBindTexture(GL_TEXTURE_1D, GetHeatmapTextureId()));
    glVerify(glUniform1i(glGetUniformLocation(s_reverseTexProgram, "colorMap"), 1));//
  }

  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, mesh->mNumVertices * sizeof(Vec4), positions));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(Vec4), mesh->mNumVertices * sizeof(Vec4), normals));
  // update texture coords
  if (textureMode > 0)
  {
    glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * (sizeof(Vec4) + sizeof(Vec4)), mesh->mNumVertices * sizeof(Vec4), uvs));
  }
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * (sizeof(Vec4) + sizeof(Vec4) + sizeof(Vec4)), mesh->mNumVertices * sizeof(Vec4), stretchColors));
  // draw VAO
  glVerify(glBindVertexArray(mesh->mVAO));

  if (g_enable_paches)
  {
    glVerify(glDrawElements(GL_PATCHES, nIndices, GL_UNSIGNED_INT, 0)); // when using tesselation shader
  }
  else
  {
    glVerify(glDrawElements(GL_TRIANGLES, nIndices, GL_UNSIGNED_INT, 0));
  }

  glVerify(glBindVertexArray(0));
  // disable texture
  if (textureMode > 0)
  {
    glActiveTexture(GL_TEXTURE0);
    glDisable(GL_TEXTURE_2D);
  }
  if (textureMode > 1)
  {
    glVerify(glActiveTexture(GL_TEXTURE1));
    glDisable(GL_TEXTURE_1D);
  }
}

GLuint* GetTexturePixels(GLuint textureID, int width, int height, int chanels)
{
  GLuint* pixels = new GLuint[width * height * chanels];
  glBindTexture(GL_TEXTURE_2D, textureID);
  //glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_INT, pixels);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

  return pixels;
}


void CreateHydrographicFilmImage(int W, int H, int imgW)
{
  FILE   *out = fopen("../../movies/texture.tga", "wb");
  char   *pixel_data = new char[3 * imgW*H];
  short  TGAhead[] = { 0, 2, 0, 0, 0, 0, imgW, H, 24 };

  //glReadBuffer(GL_BACK);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(int((W - imgW)*0.5f), 0, imgW, H, GL_BGR, GL_UNSIGNED_BYTE, pixel_data);

  fwrite(&TGAhead, sizeof(TGAhead), 1, out);
  fwrite(pixel_data, 3 * imgW*H, 1, out);
  fclose(out);

  delete[] pixel_data;
}
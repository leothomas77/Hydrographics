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

GLuint texHydrographicId;
GLuint texModelId;
int texWidth, texHeight, components;
GLuint VAO = -1;
GLuint VBO = -1;
GLuint EBO = -1;
GLuint s_diffuseProgram = GLuint(-1);
GLuint s_shadowProgram = GLuint(-1);
GLuint s_hydrographicProgram = GLuint(-1);
GLuint s_hydrographicProgramV2 = GLuint(-1);
GLuint s_displacementProgram = GLuint(-1);
GLuint s_diffuseProgramV2 = GLuint(-1);
GLuint s_filmProgram = GLuint(-1);

#ifdef ANDROID
#include "android/Log.h"
#include "android/AndroidDefine.h"
#include "android/AndroidMatrixTool.h"
#endif


#define CudaCheck(x) { cudaError_t err = x; if (err != cudaSuccess) { printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); assert(0); } }

typedef unsigned int VertexBuffer;
typedef unsigned int IndexBuffer;
typedef unsigned int Texture;

struct FluidRenderBuffersGL
{
	FluidRenderBuffersGL(int numParticles = 0):
		mPositionVBO(0),
		mDensityVBO(0),
		mIndices(0),
		mPositionBuf(nullptr),
		mDensitiesBuf(nullptr),
		mIndicesBuf(nullptr)
	{
		mNumParticles = numParticles;
		for (int i = 0; i < 3; i++)
		{ 
			mAnisotropyVBO[i] = 0;
			mAnisotropyBuf[i] = nullptr;
		}
	}
	~FluidRenderBuffersGL()
	{
		glDeleteBuffers(1, &mPositionVBO);
		glDeleteBuffers(3, mAnisotropyVBO);
		glDeleteBuffers(1, &mDensityVBO);
		glDeleteBuffers(1, &mIndices);

		NvFlexUnregisterOGLBuffer(mPositionBuf);
		NvFlexUnregisterOGLBuffer(mDensitiesBuf);
		NvFlexUnregisterOGLBuffer(mIndicesBuf);

		NvFlexUnregisterOGLBuffer(mAnisotropyBuf[0]);
		NvFlexUnregisterOGLBuffer(mAnisotropyBuf[1]);
		NvFlexUnregisterOGLBuffer(mAnisotropyBuf[2]);
	}

	int mNumParticles;
	VertexBuffer mPositionVBO;
	VertexBuffer mDensityVBO;
	VertexBuffer mAnisotropyVBO[3];
	IndexBuffer mIndices;

	// wrapper buffers that allow Flex to write directly to VBOs
	NvFlexBuffer* mPositionBuf;
	NvFlexBuffer* mDensitiesBuf;
	NvFlexBuffer* mAnisotropyBuf[3];
	NvFlexBuffer* mIndicesBuf;
};

// vertex buffers for diffuse particles
struct DiffuseRenderBuffersGL
{
	DiffuseRenderBuffersGL(int numParticles = 0):
		mDiffusePositionVBO(0),
		mDiffuseVelocityVBO(0),
		mDiffuseIndicesIBO(0),
		mDiffuseIndicesBuf(nullptr),
		mDiffusePositionsBuf(nullptr),
		mDiffuseVelocitiesBuf(nullptr)
	{
		mNumParticles = numParticles;
	}
	~DiffuseRenderBuffersGL()
	{
		if (mNumParticles > 0)
		{
			glDeleteBuffers(1, &mDiffusePositionVBO);
			glDeleteBuffers(1, &mDiffuseVelocityVBO);
			glDeleteBuffers(1, &mDiffuseIndicesIBO);

			NvFlexUnregisterOGLBuffer(mDiffuseIndicesBuf);
			NvFlexUnregisterOGLBuffer(mDiffusePositionsBuf);
			NvFlexUnregisterOGLBuffer(mDiffuseVelocitiesBuf);
		}
	}

	int mNumParticles;
	VertexBuffer mDiffusePositionVBO;
	VertexBuffer mDiffuseVelocityVBO;
	IndexBuffer mDiffuseIndicesIBO;

	NvFlexBuffer* mDiffuseIndicesBuf;
	NvFlexBuffer* mDiffusePositionsBuf;
	NvFlexBuffer* mDiffuseVelocitiesBuf;
};

struct FluidRenderer
{
	GLuint mDepthFbo;
	GLuint mDepthTex;
	GLuint mDepthSmoothTex;
	GLuint mSceneFbo;
	GLuint mSceneTex;
	GLuint mReflectTex;

	GLuint mThicknessFbo;
	GLuint mThicknessTex;

	GLuint mPointThicknessProgram;
	//GLuint mPointDepthProgram;

	GLuint mEllipsoidThicknessProgram;
	GLuint mEllipsoidDepthProgram;

	GLuint mCompositeProgram;
	GLuint mDepthBlurProgram;

	int mSceneWidth;
	int mSceneHeight;
};

struct ShadowMap
{
  GLuint texture;
  GLuint framebuffer;
};

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
  std::vector<Vec3> positions;
  std::vector<Vec3> normals;
  std::vector<Vec4> colors;
  std::vector<Vec2> texCoords;
  std::vector<Triangle> triangles;
  Matrix44 modelTransform;

  Vec4 Ka, Kd, Ks;
};

/* the global Assimp scene object */
const aiScene* scene = NULL;
GLuint scene_list = 0;
aiVector3D scene_min, scene_max, scene_center;

// texture pool
#include "../core/png.h"

GLuint LoadTexture(const char* filename)
{
    PngImage img;
    if (PngLoad(filename, img))
    {
        GLuint tex;

        glVerify(glGenTextures(1, &tex));
        glVerify(glActiveTexture(GL_TEXTURE0));
        glVerify(glBindTexture(GL_TEXTURE_2D, tex));

        glTexStorage2D(GL_TEXTURE_2D, 2 /* mip map levels */, GL_RGB8, img.m_width, img.m_height);
        glTexSubImage2D(GL_TEXTURE_2D, 0 /* mip map level */, 0 /* xoffset */, 0 /* yoffset */, img.m_width, img.m_height, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data);
        glGenerateMipmap(GL_TEXTURE_2D);


        glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
        glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
        glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
        glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));
        //glVerify(glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE));
        //glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.m_width, img.m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.m_data));

        PngFree(img);

        return tex;
    }
    else
    {
        return NULL;
    }
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

extern Mesh* g_mesh;
extern GpuMesh* g_gpu_mesh;
//void DrawShapes();

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
	  SDL_GL_SetSwapInterval(1);

	  if (!gladLoadGLLoader(SDL_GL_GetProcAddress))
	  {
		  printf("Could not initialize GL extensions\n");
	  }

	  imguiRenderGLInit(GetFilePathByPlatform("../../data/DroidSans.ttf").c_str());


    if (s_diffuseProgramV2 == GLuint(-1))
    {
      s_diffuseProgramV2 = InitShader("../../shaders/rigid.vs", "../../shaders/rigid.fs");
    }

    if (s_displacementProgram == GLuint(-1))
    {
      s_displacementProgram = InitShader("../../shaders/displacements.vs", "../../shaders/displacements.fs");
    }

	  //Load texture - begin
	  //EPRek.png xadrez7x11.png malha_rgb.jpg
	  //8k_earth_daymap.jpg
	  //malha_rgb.jpg

    texHydrographicId = LoadTexture(GetFilePathByPlatform("../../textures/malha_rgb.jpg").c_str());

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

RenderTexture* CreateRenderTexture(const char* filename)
{
	GLuint tex = LoadTexture(filename);

	if (tex)
	{
		RenderTexture* t = new RenderTexture();
		t->colorTex = tex;

		return t;
	}
	else
	{
		return NULL;
	}
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

  // vertex shader
  const char *vertexPointShader = "#version 130\n" STRINGIFY(

    uniform float pointRadius;  // point size in world space
  uniform float pointScale;   // scale to calculate size in pixels

  uniform mat4 lightTransform;
  uniform vec3 lightDir;
  uniform vec3 lightDirView;

  uniform vec4 colors[8];

  uniform vec4 transmission;
  uniform int mode;

  //in int density;
  in float density;
  in int phase;
  in vec4 velocity;

  void main()
  {
    // calculate window-space point size
    vec4 viewPos = gl_ModelViewMatrix*vec4(gl_Vertex.xyz, 1.0);

    gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1.0);
    gl_PointSize = -pointScale * (pointRadius / viewPos.z);

    gl_TexCoord[0] = gl_MultiTexCoord0;
    gl_TexCoord[1] = lightTransform*vec4(gl_Vertex.xyz - lightDir*pointRadius*2.0, 1.0);
    gl_TexCoord[2] = gl_ModelViewMatrix*vec4(lightDir, 0.0);

    if (mode == 1)
    {
      // density visualization
      if (density < 0.0f)
        gl_TexCoord[3].xyz = mix(vec3(0.1, 0.1, 1.0), vec3(0.1, 1.0, 1.0), -density);
      else
        gl_TexCoord[3].xyz = mix(vec3(1.0, 1.0, 1.0), vec3(0.1, 0.2, 1.0), density);
    }
    else if (mode == 2)
    {
      gl_PointSize *= clamp(gl_Vertex.w*0.25, 0.0f, 1.0);

      gl_TexCoord[3].xyzw = vec4(clamp(gl_Vertex.w*0.05, 0.0f, 1.0));
    }
    else
    {
      gl_TexCoord[3].xyz = mix(colors[phase % 8].xyz*2.0, vec3(1.0), 0.1);
    }

    gl_TexCoord[4].xyz = gl_Vertex.xyz;
    gl_TexCoord[5].xyz = viewPos.xyz;
  }
  );

  // pixel shader for rendering points as shaded spheres
  const char *fragmentPointShader = STRINGIFY(

    uniform vec3 lightDir;
  uniform vec3 lightPos;
  uniform float spotMin;
  uniform float spotMax;
  uniform int mode;

  uniform sampler2DShadow shadowTex;
  uniform vec2 shadowTaps[12];
  uniform float pointRadius;  // point size in world space

                              // sample shadow map
  float shadowSample()
  {
    vec3 pos = vec3(gl_TexCoord[1].xyz / gl_TexCoord[1].w);
    vec3 uvw = (pos.xyz*0.5) + vec3(0.5);

    // user clip
    if (uvw.x  < 0.0 || uvw.x > 1.0)
      return 1.0;
    if (uvw.y < 0.0 || uvw.y > 1.0)
      return 1.0;

    float s = 0.0;
    float radius = 0.002;

    for (int i = 0; i < 8; i++)
    {
      s += shadow2D(shadowTex, vec3(uvw.xy + shadowTaps[i] * radius, uvw.z)).r;
    }

    s /= 8.0;
    return s;
  }

  float sqr(float x) { return x*x; }

  void main()
  {
    // calculate normal from texture coordinates
    vec3 normal;
    normal.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
    normal.z = sqrt(1.0 - mag);

    if (mode == 2)
    {
      float alpha = normal.z*gl_TexCoord[3].w;
      gl_FragColor.xyz = gl_TexCoord[3].xyz*alpha;
      gl_FragColor.w = alpha;
      return;
    }

    // calculate lighting
    float shadow = shadowSample();

    vec3 lVec = normalize(gl_TexCoord[4].xyz - (lightPos));
    vec3 lPos = vec3(gl_TexCoord[1].xyz / gl_TexCoord[1].w);
    float attenuation = max(smoothstep(spotMax, spotMin, dot(lPos.xy, lPos.xy)), 0.05);

    vec3 diffuse = vec3(0.9, 0.9, 0.9);
    vec3 reflectance = gl_TexCoord[3].xyz;

    vec3 Lo = diffuse*reflectance*max(0.0, sqr(-dot(gl_TexCoord[2].xyz, normal)*0.5 + 0.5))*max(0.2, shadow)*attenuation;

    gl_FragColor = vec4(pow(Lo, vec3(1.0 / 2.2)), 1.0);

    vec3 eyePos = gl_TexCoord[5].xyz + normal*pointRadius;//*2.0;
    vec4 ndcPos = gl_ProjectionMatrix * vec4(eyePos, 1.0);
    ndcPos.z /= ndcPos.w;
    gl_FragDepth = ndcPos.z*0.5 + 0.5;
  }
  );

  // vertex shader
  const char *vertexShader = "#version 130\n" STRINGIFY(

    uniform mat4 lightTransform;
  uniform vec3 lightDir;
  uniform float bias;
  uniform vec4 clipPlane;
  uniform float expand;

  uniform mat4 objectTransform;

  void main()
  {
    vec3 n = normalize((objectTransform*vec4(gl_Normal, 0.0)).xyz);
    vec3 p = (objectTransform*vec4(gl_Vertex.xyz, 1.0)).xyz;

    // calculate window-space point size
    gl_Position = gl_ModelViewProjectionMatrix * vec4(p + expand*n, 1.0);

    gl_TexCoord[0].xyz = n;
    gl_TexCoord[1] = lightTransform*vec4(p + n*bias, 1.0);
    gl_TexCoord[2] = gl_ModelViewMatrix*vec4(lightDir, 0.0);
    gl_TexCoord[3].xyz = p;
    gl_TexCoord[4] = gl_Color;
    gl_TexCoord[5] = gl_MultiTexCoord0;
    gl_TexCoord[6] = gl_SecondaryColor;
    gl_TexCoord[7] = gl_ModelViewMatrix*vec4(gl_Vertex.xyz, 1.0);

    gl_ClipDistance[0] = dot(clipPlane, vec4(gl_Vertex.xyz, 1.0));
  }
  );

  const char *passThroughShader = STRINGIFY(

    void main()
  {
    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);

  }
  );

  // pixel shader for rendering points as shaded spheres
  const char *fragmentShader = STRINGIFY(

    uniform vec3 lightDir;
  uniform vec3 lightPos;
  uniform float spotMin;
  uniform float spotMax;
  uniform vec3 color;
  uniform vec4 fogColor;

  uniform sampler2DShadow shadowTex;
  uniform vec2 shadowTaps[12];

  uniform sampler2D tex;
  uniform bool sky;

  uniform bool grid;
  uniform bool texture;

  float sqr(float x) { return x*x; }

  // sample shadow map
  float shadowSample()
  {
    vec3 pos = vec3(gl_TexCoord[1].xyz / gl_TexCoord[1].w);
    vec3 uvw = (pos.xyz*0.5) + vec3(0.5);

    // user clip
    if (uvw.x  < 0.0 || uvw.x > 1.0)
      return 1.0;
    if (uvw.y < 0.0 || uvw.y > 1.0)
      return 1.0;

    float s = 0.0;
    float radius = 0.002;

    const int numTaps = 12;

    for (int i = 0; i < numTaps; i++)
    {
      s += shadow2D(shadowTex, vec3(uvw.xy + shadowTaps[i] * radius, uvw.z)).r;
    }

    s /= numTaps;
    return s;
  }

  float filterwidth(vec2 v)
  {
    vec2 fw = max(abs(dFdx(v)), abs(dFdy(v)));
    return max(fw.x, fw.y);
  }

  vec2 bump(vec2 x)
  {
    return (floor((x) / 2) + 2.f * max(((x) / 2) - floor((x) / 2) - .5f, 0.f));
  }

  float checker(vec2 uv)
  {
    float width = filterwidth(uv);
    vec2 p0 = uv - 0.5 * width;
    vec2 p1 = uv + 0.5 * width;

    vec2 i = (bump(p1) - bump(p0)) / width;
    return i.x * i.y + (1 - i.x) * (1 - i.y);
  }

  void main()
  {
    // calculate lighting
    float shadow = max(shadowSample(), 0.5);

    vec3 lVec = normalize(gl_TexCoord[3].xyz - (lightPos));
    vec3 lPos = vec3(gl_TexCoord[1].xyz / gl_TexCoord[1].w);
    float attenuation = max(smoothstep(spotMax, spotMin, dot(lPos.xy, lPos.xy)), 0.05);

    vec3 n = gl_TexCoord[0].xyz;
    vec3 color = gl_TexCoord[4].xyz;

    if (!gl_FrontFacing)
    {
      color = gl_TexCoord[6].xyz;
      n *= -1.0f;
    }

    if (grid && (n.y >0.995))
    {
      color *= 1.0 - 0.25 * checker(vec2(gl_TexCoord[3].x, gl_TexCoord[3].z));
    }
    else if (grid && abs(n.z) > 0.995)
    {
      color *= 1.0 - 0.25 * checker(vec2(gl_TexCoord[3].y, gl_TexCoord[3].x));
    }

    if (texture)
    {
      color = texture2D(tex, gl_TexCoord[5].xy).xyz;
    }

    // direct light term
    float wrap = 0.0;
    vec3 diffuse = color*vec3(1.0, 1.0, 1.0)*max(0.0, (-dot(lightDir, n) + wrap) / (1.0 + wrap)*shadow)*attenuation;

    // wrap ambient term aligned with light dir
    vec3 light = vec3(0.03, 0.025, 0.025)*1.5;
    vec3 dark = vec3(0.025, 0.025, 0.03);
    vec3 ambient = 4.0*color*mix(dark, light, -dot(lightDir, n)*0.5 + 0.5)*attenuation;

    vec3 fog = mix(vec3(fogColor), diffuse + ambient, exp(gl_TexCoord[7].z*fogColor.w));

    gl_FragColor = vec4(pow(fog, vec3(1.0 / 2.2)), 1.0);
  }
  );

  const char *vertexHydrographicShader = "#version 130\n" STRINGIFY(

    uniform mat4 lightTransform;
  uniform vec3 lightDir;
  uniform float bias;
  uniform vec4 clipPlane;
  uniform float expand;

  uniform mat4 objectTransform;

  void main()
  {
    vec3 n = normalize((objectTransform*vec4(gl_Normal, 0.0)).xyz);
    vec3 p = (objectTransform*vec4(gl_Vertex.xyz, 1.0)).xyz;

    // calculate window-space point size
    gl_Position = gl_ModelViewProjectionMatrix * vec4(p + expand*n, 1.0);

    gl_TexCoord[0].xyz = n;
    gl_TexCoord[1] = lightTransform*vec4(p + n*bias, 1.0);
    gl_TexCoord[2] = gl_ModelViewMatrix*vec4(lightDir, 0.0);
    gl_TexCoord[3].xyz = p;
    gl_TexCoord[4] = gl_Color;
    gl_TexCoord[5] = gl_MultiTexCoord1; //gl_MultiTexCoord0 used for shadow pass
    gl_TexCoord[6] = gl_SecondaryColor;
    gl_TexCoord[7] = gl_ModelViewMatrix*vec4(gl_Vertex.xyz, 1.0);

    gl_ClipDistance[0] = dot(clipPlane, vec4(gl_Vertex.xyz, 1.0));
  }

  );

  const char *fragmentHydrographicShader = STRINGIFY(

    uniform vec3 lightDir;
  uniform vec3 lightPos;
  uniform float spotMin;
  uniform float spotMax;
  uniform vec3 color;
  uniform vec4 fogColor;

  uniform sampler2DShadow shadowTex;
  uniform vec2 shadowTaps[12];

  uniform sampler2D tex;

  uniform bool sky;

  uniform bool grid;
  uniform bool showTexture;

  float sqr(float x) { return x*x; }

  // sample shadow map
  float shadowSample()
  {
    vec3 pos = vec3(gl_TexCoord[1].xyz / gl_TexCoord[1].w);
    vec3 uvw = (pos.xyz*0.5) + vec3(0.5);

    // user clip
    if (uvw.x  < 0.0 || uvw.x > 1.0)
      return 1.0;
    if (uvw.y < 0.0 || uvw.y > 1.0)
      return 1.0;

    float s = 0.0;
    float radius = 0.002;

    const int numTaps = 12;

    for (int i = 0; i < numTaps; i++)
    {
      s += shadow2D(shadowTex, vec3(uvw.xy + shadowTaps[i] * radius, uvw.z)).r;
    }

    s /= numTaps;
    return s;
  }

  float filterwidth(vec2 v)
  {
    vec2 fw = max(abs(dFdx(v)), abs(dFdy(v)));
    return max(fw.x, fw.y);
  }

  vec2 bump(vec2 x)
  {
    return (floor((x) / 2) + 2.f * max(((x) / 2) - floor((x) / 2) - .5f, 0.f));
  }

  float checker(vec2 uv)
  {
    float width = filterwidth(uv);
    vec2 p0 = uv - 0.5 * width;
    vec2 p1 = uv + 0.5 * width;

    vec2 i = (bump(p1) - bump(p0)) / width;
    return i.x * i.y + (1 - i.x) * (1 - i.y);
  }

  void main()
  {
    // calculate lighting
    float shadow = max(shadowSample(), 0.5);

    vec3 lVec = normalize(gl_TexCoord[3].xyz - (lightPos));
    vec3 lPos = vec3(gl_TexCoord[1].xyz / gl_TexCoord[1].w);
    float attenuation = max(smoothstep(spotMax, spotMin, dot(lPos.xy, lPos.xy)), 0.05);

    vec3 n = gl_TexCoord[0].xyz;
    vec3 color = gl_TexCoord[4].xyz;

    if (!gl_FrontFacing)
    {
      color = gl_TexCoord[6].xyz;
      n *= -1.0f;
    }

    if (grid && (n.y > 0.995))
    {
      color *= 1.0 - 0.25 * checker(vec2(gl_TexCoord[3].x, gl_TexCoord[3].z));
    }
    else if (grid && abs(n.z) > 0.995)
    {
      color *= 1.0 - 0.25 * checker(vec2(gl_TexCoord[3].y, gl_TexCoord[3].x));
    }

    if (showTexture)
    {
      color = texture2D(tex, gl_TexCoord[5].xy).xyz;
    }

    // direct light term
    float wrap = 0.0;
    vec3 diffuse = color*vec3(1.0, 1.0, 1.0)*max(0.0, (-dot(lightDir, n) + wrap) / (1.0 + wrap)*shadow)*attenuation;

    // wrap ambient term aligned with light dir
    vec3 light = vec3(0.03, 0.025, 0.025)*1.5;
    vec3 dark = vec3(0.025, 0.025, 0.03);
    vec3 ambient = 4.0*color*mix(dark, light, -dot(lightDir, n)*0.5 + 0.5)*attenuation;

    vec3 fog = mix(vec3(fogColor), diffuse + ambient, exp(gl_TexCoord[7].z*fogColor.w));

    gl_FragColor = vec4(pow(fog, vec3(1.0 / 2.2)), 1.0);
  }

  );


 void ShadowApply(GLint sprogram, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, GLuint shadowTex)
{
	GLint uLightTransform = glGetUniformLocation(sprogram, "lightTransform");
	glUniformMatrix4fv(uLightTransform, 1, false, lightTransform);

	GLint uLightPos = glGetUniformLocation(sprogram, "lightPos");
	glUniform3fv(uLightPos, 1, lightPos);

	GLint uLightDir = glGetUniformLocation(sprogram, "lightDir");
	glUniform3fv(uLightDir, 1, Normalize(lightTarget - lightPos));

	GLint uBias = glGetUniformLocation(sprogram, "bias");
	glUniform1f(uBias, g_shadowBias);

	const Vec2 taps[] =
	{
		Vec2(-0.326212f,-0.40581f),Vec2(-0.840144f,-0.07358f),
		Vec2(-0.695914f,0.457137f),Vec2(-0.203345f,0.620716f),
		Vec2(0.96234f,-0.194983f),Vec2(0.473434f,-0.480026f),
		Vec2(0.519456f,0.767022f),Vec2(0.185461f,-0.893124f),
		Vec2(0.507431f,0.064425f),Vec2(0.89642f,0.412458f),
		Vec2(-0.32194f,-0.932615f),Vec2(-0.791559f,-0.59771f)
	};

	GLint uShadowTaps = glGetUniformLocation(sprogram, "shadowTaps");
	glUniform2fv(uShadowTaps, 12, &taps[0].x);

	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, shadowTex);

  }

  void DrawPoints(FluidRenderBuffers* buffersIn, int n, int offset, float radius, float screenWidth, float screenAspect, float fov, Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ShadowMap* shadowMap, bool showDensity)
  {
    FluidRenderBuffersGL* buffers = reinterpret_cast<FluidRenderBuffersGL*>(buffersIn);
    GLuint positions = buffers->mPositionVBO;
    GLuint colors = buffers->mDensityVBO;
    GLuint indices = buffers->mIndices;

    static int sprogram = -1;
    if (sprogram == -1)
    {
      sprogram = CompileProgram(vertexPointShader, fragmentPointShader);
    }

    if (sprogram)
    {
      glEnable(GL_POINT_SPRITE);
      glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
      glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
      //glDepthMask(GL_TRUE);
      glEnable(GL_DEPTH_TEST);

      int mode = 0;
      if (showDensity)
        mode = 1;
      if (shadowMap == NULL)
        mode = 2;

      glVerify(glUseProgram(sprogram));
      glVerify(glUniform1f(glGetUniformLocation(sprogram, "pointRadius"), radius));
      glVerify(glUniform1f(glGetUniformLocation(sprogram, "pointScale"), screenWidth / screenAspect * (1.0f / (tanf(fov*0.5f)))));
      glVerify(glUniform1f(glGetUniformLocation(sprogram, "spotMin"), g_spotMin));
      glVerify(glUniform1f(glGetUniformLocation(sprogram, "spotMax"), g_spotMax));
      glVerify(glUniform1i(glGetUniformLocation(sprogram, "mode"), mode));
      glVerify(glUniform4fv(glGetUniformLocation(sprogram, "colors"), 8, (float*)&g_colors[0].r));

      // set shadow parameters
      ShadowApply(sprogram, lightPos, lightTarget, lightTransform, shadowMap->texture);

      glEnableClientState(GL_VERTEX_ARRAY);
      glBindBuffer(GL_ARRAY_BUFFER, positions);
      glVertexPointer(4, GL_FLOAT, 0, 0);

      int d = glGetAttribLocation(sprogram, "density");
      int p = glGetAttribLocation(sprogram, "phase");

      if (d != -1)
      {
        glVerify(glEnableVertexAttribArray(d));
        glVerify(glBindBuffer(GL_ARRAY_BUFFER, colors));
        glVerify(glVertexAttribPointer(d, 1, GL_FLOAT, GL_FALSE, 0, 0));	// densities
      }

      if (p != -1)
      {
        glVerify(glEnableVertexAttribArray(p));
        glVerify(glBindBuffer(GL_ARRAY_BUFFER, colors));
        glVerify(glVertexAttribIPointer(p, 1, GL_INT, 0, 0));			// phases
      }

      glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices));

      glVerify(glDrawElements(GL_POINTS, n, GL_UNSIGNED_INT, (const void*)(offset * sizeof(int))));

      glVerify(glUseProgram(0));
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
      glVerify(glDisableClientState(GL_VERTEX_ARRAY));

      if (d != -1)
        glVerify(glDisableVertexAttribArray(d));
      if (p != -1)
        glVerify(glDisableVertexAttribArray(p));

      glDisable(GL_POINT_SPRITE);
      glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    }
  }

  //void DrawPlane(const Vec4& p);

  static GLuint s_diffuseProgram = GLuint(-1);
  static GLuint s_shadowProgram = GLuint(-1);

#ifdef ANDROID
  void ResetProgramId()
  {
    s_diffuseProgram = GLuint(-1);
    s_shadowProgram = GLuint(-1);
  }
#endif

  static const int kShadowResolution = 2048;

  ShadowMap* ShadowCreate()
  {
    GLuint texture;
    GLuint framebuffer;

    glVerify(glGenFramebuffers(1, &framebuffer));
    glVerify(glGenTextures(1, &texture));
    glVerify(glBindTexture(GL_TEXTURE_2D, texture));

    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    // This is to allow usage of shadow2DProj function in the shader 
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY));

    glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, kShadowResolution, kShadowResolution, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL));

    glVerify(glBindFramebuffer(GL_FRAMEBUFFER, framebuffer));

    glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texture, 0));

    ShadowMap* map = new ShadowMap();
    map->texture = texture;
    map->framebuffer = framebuffer;

    return map;

  }

  void ShadowDestroy(ShadowMap* map)
  {
    glVerify(glDeleteTextures(1, &map->texture));
    glVerify(glDeleteFramebuffers(1, &map->framebuffer));

    delete map;
  }

  void ShadowBegin(ShadowMap* map)
  {
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(8.f, 8.f);

    glVerify(glBindFramebuffer(GL_FRAMEBUFFER, map->framebuffer));

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, kShadowResolution, kShadowResolution);

    // draw back faces (for teapot)
    glDisable(GL_CULL_FACE);

    // bind shadow shader
    if (s_shadowProgram == GLuint(-1))
      s_shadowProgram = CompileProgram(vertexShader, passThroughShader);

    glUseProgram(s_shadowProgram);
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_shadowProgram, "objectTransform"), 1, false, Matrix44::kIdentity));
  }

  void ShadowEnd()
  {
    glDisable(GL_POLYGON_OFFSET_FILL);

    glVerify(glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo));

    glEnable(GL_CULL_FACE);
    glUseProgram(0);
  }

  void BindSolidShader(Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ShadowMap* shadowMap, float bias, Vec4 fogColor)
  {
  	glVerify(glViewport(0, 0, g_screenWidth, g_screenHeight));
  
  	if (s_diffuseProgram == GLuint(-1))
  		s_diffuseProgram = CompileProgram(vertexShader, fragmentShader);
  
  	if (s_diffuseProgram)
  	{
  		glDepthMask(GL_TRUE);
  		glEnable(GL_DEPTH_TEST);
  
  		glVerify(glUseProgram(s_diffuseProgram));
  		glVerify(glUniform1i(glGetUniformLocation(s_diffuseProgram, "grid"), 0));
  		glVerify(glUniform1f( glGetUniformLocation(s_diffuseProgram, "spotMin"), g_spotMin));
  		glVerify(glUniform1f( glGetUniformLocation(s_diffuseProgram, "spotMax"), g_spotMax));
  		glVerify(glUniform4fv(glGetUniformLocation(s_diffuseProgram, "fogColor"), 1, fogColor));
  
  		glVerify(glUniformMatrix4fv(glGetUniformLocation(s_diffuseProgram, "objectTransform"), 1, false, Matrix44::kIdentity));
  
  		// set shadow parameters
  		ShadowApply(s_diffuseProgram, lightPos, lightTarget, lightTransform, shadowMap->texture);
  	}
  }

  void UnbindSolidShader()
  {
    glActiveTexture(GL_TEXTURE1);
    glDisable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);

    glUseProgram(0);
  }


  void SetMaterial(const Matrix44& xform, const RenderMaterial& mat)
  {
    GLint program;
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);

    if (program)
    {
      glUniformMatrix4fv(glGetUniformLocation(program, "objectTransform"), 1, false, xform);

      const float maxSpecularPower = 2048.0f;

      glVerify(glUniform1f(glGetUniformLocation(program, "specularPower"), powf(maxSpecularPower, 1.0f - mat.roughness)));
      glVerify(glUniform3fv(glGetUniformLocation(program, "specularColor"), 1, Lerp(Vec3(mat.specular*0.08f), mat.frontColor, mat.metallic)));
      glVerify(glUniform1f(glGetUniformLocation(program, "roughness"), mat.roughness));
      glVerify(glUniform1f(glGetUniformLocation(program, "metallic"), mat.metallic));

      // set material properties
      if (mat.colorTex)
      {
        GLuint tex = mat.colorTex->colorTex;

        glActiveTexture(GL_TEXTURE1);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, tex);

        glVerify(glUniform1i(glGetUniformLocation(program, "tex"), 1));		// use slot one
        glVerify(glUniform1i(glGetUniformLocation(program, "texture"), 1)); // enable tex sampling
      }
      else
      {
        glVerify(glUniform1i(glGetUniformLocation(program, "tex"), 1));		// use slot one
        glVerify(glUniform1i(glGetUniformLocation(program, "texture"), 0)); // disable tex sampling}
      }
    }

    glVerify(glColor3fv(mat.frontColor));
    glVerify(glSecondaryColor3fv(mat.backColor));
  }


  void DrawPlanes(Vec4* planes, int n, float bias)
  {
    // diffuse 		
    glColor3f(0.9f, 0.9f, 0.9f);

    GLint uBias = glGetUniformLocation(s_diffuseProgram, "bias");
    glVerify(glUniform1f(uBias, 0.0f));
    GLint uGrid = glGetUniformLocation(s_diffuseProgram, "grid");
    glVerify(glUniform1i(uGrid, 1));
    GLint uExpand = glGetUniformLocation(s_diffuseProgram, "expand");
    glVerify(glUniform1f(uExpand, 0.0f));

    for (int i = 0; i < n; ++i)
    {
      Vec4 p = planes[i];
      p.w -= bias;

      DrawPlane(p, false);
    }

    glVerify(glUniform1i(uGrid, 0));
    glVerify(glUniform1f(uBias, g_shadowBias));
  }

  void DrawMesh(const Mesh* m, Vec3 color)
  {
    if (m)
    {
      glVerify(glColor3fv(color));
      glVerify(glSecondaryColor3fv(color));

      glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
      glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

      glVerify(glEnableClientState(GL_NORMAL_ARRAY));
      glVerify(glEnableClientState(GL_VERTEX_ARRAY));

      glVerify(glNormalPointer(GL_FLOAT, sizeof(float) * 3, &m->m_normals[0]));
      glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float) * 3, &m->m_positions[0]));

      if (m->m_colours.size())
      {
        glVerify(glEnableClientState(GL_COLOR_ARRAY));
        glVerify(glColorPointer(4, GL_FLOAT, 0, &m->m_colours[0]));
      }

      glVerify(glDrawElements(GL_TRIANGLES, m->GetNumFaces() * 3, GL_UNSIGNED_INT, &m->m_indices[0]));

      glVerify(glDisableClientState(GL_VERTEX_ARRAY));
      glVerify(glDisableClientState(GL_NORMAL_ARRAY));

      if (m->m_colours.size())
        glVerify(glDisableClientState(GL_COLOR_ARRAY));
    }
  }

  void DrawCloth(const Vec4* positions, const Vec4* normals, const float* uvs, const int* indices, int numTris, int numPositions, int colorIndex, float expand, bool twosided, bool smooth)
  {
    if (!numTris)
      return;

    if (twosided)
      glDisable(GL_CULL_FACE);

#if 1
    GLint program;
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);

    if (program == GLint(s_diffuseProgram))
    {
      GLint uBias = glGetUniformLocation(s_diffuseProgram, "bias");
      glUniform1f(uBias, 0.0f);

      GLint uExpand = glGetUniformLocation(s_diffuseProgram, "expand");
      glUniform1f(uExpand, expand);
    }
#endif

    glColor3fv(g_colors[colorIndex + 1] * 1.5f);
    glSecondaryColor3fv(g_colors[colorIndex] * 1.5f);

    glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
    glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

    glVerify(glEnableClientState(GL_VERTEX_ARRAY));
    glVerify(glEnableClientState(GL_NORMAL_ARRAY));

    glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float) * 4, positions));
    glVerify(glNormalPointer(GL_FLOAT, sizeof(float) * 4, normals));

    glVerify(glDrawElements(GL_TRIANGLES, numTris * 3, GL_UNSIGNED_INT, indices));

    glVerify(glDisableClientState(GL_VERTEX_ARRAY));
    glVerify(glDisableClientState(GL_NORMAL_ARRAY));

    if (twosided)
      glEnable(GL_CULL_FACE);

#if 1
    if (program == GLint(s_diffuseProgram))
    {
      GLint uBias = glGetUniformLocation(s_diffuseProgram, "bias");
      glUniform1f(uBias, g_shadowBias);

      GLint uExpand = glGetUniformLocation(s_diffuseProgram, "expand");
      glUniform1f(uExpand, 0.0f);
    }
#endif
  }

  void DrawRope(Vec4* positions, int* indices, int numIndices, float radius, int color)
  {
    if (numIndices < 2)
      return;

    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    std::vector<int> triangles;

    // flatten curve
    std::vector<Vec3> curve(numIndices);
    for (int i = 0; i < numIndices; ++i)
      curve[i] = Vec3(positions[indices[i]]);

    const int resolution = 8;
    const int smoothing = 3;

    vertices.reserve(resolution*numIndices*smoothing);
    normals.reserve(resolution*numIndices*smoothing);
    triangles.reserve(numIndices*resolution * 6 * smoothing);

    Extrude(&curve[0], int(curve.size()), vertices, normals, triangles, radius, resolution, smoothing);

    glVerify(glDisable(GL_CULL_FACE));
    glVerify(glColor3fv(g_colors[color % 8] * 1.5f));
    glVerify(glSecondaryColor3fv(g_colors[color % 8] * 1.5f));

    glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));

    glVerify(glEnableClientState(GL_VERTEX_ARRAY));
    glVerify(glEnableClientState(GL_NORMAL_ARRAY));

    glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float) * 3, &vertices[0]));
    glVerify(glNormalPointer(GL_FLOAT, sizeof(float) * 3, &normals[0]));

    glVerify(glDrawElements(GL_TRIANGLES, GLsizei(triangles.size()), GL_UNSIGNED_INT, &triangles[0]));

    glVerify(glDisableClientState(GL_VERTEX_ARRAY));
    glVerify(glDisableClientState(GL_NORMAL_ARRAY));
    glVerify(glEnable(GL_CULL_FACE));

  }


  struct ReflectMap
  {
    GLuint texture;

    int width;
    int height;
  };

  ReflectMap* ReflectCreate(int width, int height)
  {
    GLuint texture;

    // copy frame buffer to texture
    glVerify(glActiveTexture(GL_TEXTURE0));
    glVerify(glEnable(GL_TEXTURE_2D));

    glVerify(glGenTextures(1, &texture));
    glVerify(glBindTexture(GL_TEXTURE_2D, texture));

    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

    glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));

    ReflectMap* map = new ReflectMap();
    map->texture = texture;
    map->width = width;
    map->height = height;

    return map;
  }

  void ReflectDestroy(ReflectMap* map)
  {
    glVerify(glDeleteTextures(1, &map->texture));

    delete map;
  }

  void ReflectBegin(ReflectMap* map, Vec4 plane, int width, int height)
  {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, width, height);

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Matrix44 scale = Matrix44::kIdentity;
    scale.columns[0][0] *= -2.0f;
    scale.columns[1][1] *= -2.0f;
    scale.columns[2][2] *= -2.0f;
    scale.columns[3][3] *= -2.0f;

    Matrix44 reflect = (scale*Outer(Vec4(plane.x, plane.y, plane.z, 0.0f), plane));
    reflect.columns[0][0] += 1.0f;
    reflect.columns[1][1] += 1.0f;
    reflect.columns[2][2] += 1.0f;
    reflect.columns[3][3] += 1.0f;

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glMultMatrixf(reflect);

    glVerify(glFrontFace(GL_CW));
    glVerify(glEnable(GL_CLIP_PLANE0));

    glVerify(glUniform4fv(glGetUniformLocation(s_diffuseProgram, "clipPlane"), 1, plane));
  }

  void ReflectEnd(ReflectMap* map, int width, int height)
  {
    // copy frame buffer to texture
    glVerify(glActiveTexture(GL_TEXTURE0));
    glVerify(glEnable(GL_TEXTURE_2D));
    glVerify(glBindTexture(GL_TEXTURE_2D, map->texture));

    glVerify(glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, width, height));

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glVerify(glDisable(GL_CLIP_PLANE0));
    glVerify(glFrontFace(GL_CCW));

    glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo);

    glViewport(0, 0, g_screenWidth, g_screenHeight);
  }


  //-----------------------------------------------------------------------------------------------------
  // vertex shader

  const char *vertexPointDepthShader = STRINGIFY(

    uniform float pointRadius;  // point size in world space
  uniform float pointScale;   // scale to calculate size in pixels

  void main()
  {
    // calculate window-space point size
    gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1.0);
    gl_PointSize = pointScale * (pointRadius / gl_Position.w);

    gl_TexCoord[0] = gl_MultiTexCoord0;
    gl_TexCoord[1] = gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0);
  }
  );

  // pixel shader for rendering points as shaded spheres
  const char *fragmentPointDepthShader = STRINGIFY(

    uniform float pointRadius;  // point size in world space

  void main()
  {
    // calculate normal from texture coordinates
    vec3 normal;
    normal.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
    normal.z = sqrt(1.0 - mag);

    vec3 eyePos = gl_TexCoord[1].xyz + normal*pointRadius*2.0;
    vec4 ndcPos = gl_ProjectionMatrix * vec4(eyePos, 1.0);
    ndcPos.z /= ndcPos.w;

    gl_FragColor = vec4(eyePos.z, 1.0, 1.0, 1.0);
    gl_FragDepth = ndcPos.z*0.5 + 0.5;
  }
  );


  // pixel shader for rendering points density
  const char *fragmentPointThicknessShader = STRINGIFY(

    void main()
  {
    // calculate normal from texture coordinates
    vec3 normal;
    normal.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
    normal.z = sqrt(1.0 - mag);

    gl_FragColor = vec4(normal.z*0.005);
  }
  );

  //--------------------------------------------------------
  // Ellipsoid shaders
  //
  const char *vertexEllipsoidDepthShader = "#version 120\n" STRINGIFY(

    // rotation matrix in xyz, scale in w
    attribute vec4 q1;
  attribute vec4 q2;
  attribute vec4 q3;

  // returns 1.0 for x==0.0 (unlike glsl)
  float Sign(float x) { return x < 0.0 ? -1.0 : 1.0; }

  bool solveQuadratic(float a, float b, float c, out float minT, out float maxT)
  {
    if (a == 0.0 && b == 0.0)
    {
      minT = maxT = 0.0;
      return false;
    }

    float discriminant = b*b - 4.0*a*c;

    if (discriminant < 0.0)
    {
      return false;
    }

    float t = -0.5*(b + Sign(b)*sqrt(discriminant));
    minT = t / a;
    maxT = c / t;

    if (minT > maxT)
    {
      float tmp = minT;
      minT = maxT;
      maxT = tmp;
    }

    return true;
  }

  float DotInvW(vec4 a, vec4 b) { return a.x*b.x + a.y*b.y + a.z*b.z - a.w*b.w; }

  void main()
  {
    vec3 worldPos = gl_Vertex.xyz;// - vec3(0.0, 0.1*0.25, 0.0);	// hack move towards ground to account for anisotropy

                                  // construct quadric matrix
    mat4 q;
    q[0] = vec4(q1.xyz*q1.w, 0.0);
    q[1] = vec4(q2.xyz*q2.w, 0.0);
    q[2] = vec4(q3.xyz*q3.w, 0.0);
    q[3] = vec4(worldPos, 1.0);

    // transforms a normal to parameter space (inverse transpose of (q*modelview)^-T)
    mat4 invClip = transpose(gl_ModelViewProjectionMatrix*q);

    // solve for the right hand bounds in homogenous clip space
    float a1 = DotInvW(invClip[3], invClip[3]);
    float b1 = -2.0f*DotInvW(invClip[0], invClip[3]);
    float c1 = DotInvW(invClip[0], invClip[0]);

    float xmin;
    float xmax;
    solveQuadratic(a1, b1, c1, xmin, xmax);

    // solve for the right hand bounds in homogenous clip space
    float a2 = DotInvW(invClip[3], invClip[3]);
    float b2 = -2.0f*DotInvW(invClip[1], invClip[3]);
    float c2 = DotInvW(invClip[1], invClip[1]);

    float ymin;
    float ymax;
    solveQuadratic(a2, b2, c2, ymin, ymax);

    gl_Position = vec4(worldPos.xyz, 1.0);
    gl_TexCoord[0] = vec4(xmin, xmax, ymin, ymax);

    // construct inverse quadric matrix (used for ray-casting in parameter space)
    mat4 invq;
    invq[0] = vec4(q1.xyz / q1.w, 0.0);
    invq[1] = vec4(q2.xyz / q2.w, 0.0);
    invq[2] = vec4(q3.xyz / q3.w, 0.0);
    invq[3] = vec4(0.0, 0.0, 0.0, 1.0);

    invq = transpose(invq);
    invq[3] = -(invq*gl_Position);

    // transform a point from view space to parameter space
    invq = invq*gl_ModelViewMatrixInverse;

    // pass down
    gl_TexCoord[1] = invq[0];
    gl_TexCoord[2] = invq[1];
    gl_TexCoord[3] = invq[2];
    gl_TexCoord[4] = invq[3];

    // compute ndc pos for frustrum culling in GS
    vec4 ndcPos = gl_ModelViewProjectionMatrix * vec4(worldPos.xyz, 1.0);
    gl_TexCoord[5] = ndcPos / ndcPos.w;
  }
  );

  const char* geometryEllipsoidDepthShader =
    "#version 120\n"
    "#extension GL_EXT_geometry_shader4 : enable\n"
    STRINGIFY(
      void main()
  {
    vec3 pos = gl_PositionIn[0].xyz;
    vec4 bounds = gl_TexCoordIn[0][0];
    vec4 ndcPos = gl_TexCoordIn[0][5];

    // frustrum culling
    const float ndcBound = 1.0;
    if (ndcPos.x < -ndcBound) return;
    if (ndcPos.x > ndcBound) return;
    if (ndcPos.y < -ndcBound) return;
    if (ndcPos.y > ndcBound) return;

    float xmin = bounds.x;
    float xmax = bounds.y;
    float ymin = bounds.z;
    float ymax = bounds.w;

    // inv quadric transform
    gl_TexCoord[0] = gl_TexCoordIn[0][1];
    gl_TexCoord[1] = gl_TexCoordIn[0][2];
    gl_TexCoord[2] = gl_TexCoordIn[0][3];
    gl_TexCoord[3] = gl_TexCoordIn[0][4];

    gl_Position = vec4(xmin, ymax, 0.0, 1.0);
    EmitVertex();

    gl_Position = vec4(xmin, ymin, 0.0, 1.0);
    EmitVertex();

    gl_Position = vec4(xmax, ymax, 0.0, 1.0);
    EmitVertex();

    gl_Position = vec4(xmax, ymin, 0.0, 1.0);
    EmitVertex();
  }
  );

  // pixel shader for rendering points as shaded spheres
  const char *fragmentEllipsoidDepthShader = "#version 120\n" STRINGIFY(

    uniform vec3 invViewport;
  uniform vec3 invProjection;

  float Sign(float x) { return x < 0.0 ? -1.0 : 1.0; }

  bool solveQuadratic(float a, float b, float c, out float minT, out float maxT)
  {
    if (a == 0.0 && b == 0.0)
    {
      minT = maxT = 0.0;
      return true;
    }

    float discriminant = b*b - 4.0*a*c;

    if (discriminant < 0.0)
    {
      return false;
    }

    float t = -0.5*(b + Sign(b)*sqrt(discriminant));
    minT = t / a;
    maxT = c / t;

    if (minT > maxT)
    {
      float tmp = minT;
      minT = maxT;
      maxT = tmp;
    }

    return true;
  }

  float sqr(float x) { return x*x; }

  void main()
  {
    // transform from view space to parameter space
    mat4 invQuadric;
    invQuadric[0] = gl_TexCoord[0];
    invQuadric[1] = gl_TexCoord[1];
    invQuadric[2] = gl_TexCoord[2];
    invQuadric[3] = gl_TexCoord[3];

    vec4 ndcPos = vec4(gl_FragCoord.xy*invViewport.xy*vec2(2.0, 2.0) - vec2(1.0, 1.0), -1.0, 1.0);
    vec4 viewDir = gl_ProjectionMatrixInverse*ndcPos;

    // ray to parameter space
    vec4 dir = invQuadric*vec4(viewDir.xyz, 0.0);
    vec4 origin = invQuadric[3];

    // set up quadratric equation
    float a = sqr(dir.x) + sqr(dir.y) + sqr(dir.z);// - sqr(dir.w);
    float b = dir.x*origin.x + dir.y*origin.y + dir.z*origin.z - dir.w*origin.w;
    float c = sqr(origin.x) + sqr(origin.y) + sqr(origin.z) - sqr(origin.w);

    float minT;
    float maxT;

    if (solveQuadratic(a, 2.0*b, c, minT, maxT))
    {
      vec3 eyePos = viewDir.xyz*minT;
      vec4 ndcPos = gl_ProjectionMatrix * vec4(eyePos, 1.0);
      ndcPos.z /= ndcPos.w;

      gl_FragColor = vec4(eyePos.z, 1.0, 1.0, 1.0);
      gl_FragDepth = ndcPos.z*0.5 + 0.5;

      return;
    }
    else
      discard;

    gl_FragColor = vec4(0.5, 0.0, 0.0, 1.0);
  }
  );

  //--------------------------------------------------------------------------------
  // Composite shaders

  const char* vertexPassThroughShader = STRINGIFY(

    void main()
  {
    gl_Position = vec4(gl_Vertex.xyz, 1.0);
    gl_TexCoord[0] = gl_MultiTexCoord0;
  }
  );

  const char* fragmentBlurDepthShader =
    "#extension GL_ARB_texture_rectangle : enable\n"
    STRINGIFY(

      uniform sampler2DRect depthTex;
  uniform sampler2D thicknessTex;
  uniform float blurRadiusWorld;
  uniform float blurScale;
  uniform float blurFalloff;
  uniform vec2 invTexScale;

  uniform bool debug;

  float sqr(float x) { return x*x; }

  void main()
  {
    // eye-space depth of center sample
    float depth = texture2DRect(depthTex, gl_FragCoord.xy).x;
    float thickness = texture2D(thicknessTex, gl_TexCoord[0].xy).x;

    // hack: ENABLE_SIMPLE_FLUID
    //thickness = 0.0f;

    if (debug)
    {
      // do not blur
      gl_FragColor.x = depth;
      return;
    }

    // threshold on thickness to create nice smooth silhouettes
    if (depth == 0.0)//|| thickness < 0.02f)
    {
      gl_FragColor.x = 0.0;
      return;
    }

    /*
    float dzdx = dFdx(depth);
    float dzdy = dFdy(depth);

    // handle edge case
    if (max(abs(dzdx), abs(dzdy)) > 0.05)
    {
    dzdx = 0.0;
    dzdy = 0.0;

    gl_FragColor.x = depth;
    return;
    }
    */

    float blurDepthFalloff = 5.5;//blurFalloff*mix(4.0, 1.0, thickness)/blurRadiusWorld*0.0375;	// these constants are just a re-scaling from some known good values

    float maxBlurRadius = 5.0;
    //float taps = min(maxBlurRadius, blurScale * (blurRadiusWorld / -depth));
    //vec2 blurRadius = min(mix(0.25, 2.0/blurFalloff, thickness) * blurScale * (blurRadiusWorld / -depth) / taps, 0.15)*invTexScale;

    //discontinuities between different tap counts are visible. to avoid this we 
    //use fractional contributions between #taps = ceil(radius) and floor(radius) 
    float radius = min(maxBlurRadius, blurScale * (blurRadiusWorld / -depth));
    float radiusInv = 1.0 / radius;
    float taps = ceil(radius);
    float frac = taps - radius;

    float sum = 0.0;
    float wsum = 0.0;
    float count = 0.0;

    for (float y = -taps; y <= taps; y += 1.0)
    {
      for (float x = -taps; x <= taps; x += 1.0)
      {
        vec2 offset = vec2(x, y);

        float sample = texture2DRect(depthTex, gl_FragCoord.xy + offset).x;

        if (sample < -10000.0*0.5)
          continue;

        // spatial domain
        float r1 = length(vec2(x, y))*radiusInv;
        float w = exp(-(r1*r1));

        //float expectedDepth = depth + dot(vec2(dzdx, dzdy), offset);

        // range domain (based on depth difference)
        float r2 = (sample - depth) * blurDepthFalloff;
        float g = exp(-(r2*r2));

        //fractional radius contributions
        float wBoundary = step(radius, max(abs(x), abs(y)));
        float wFrac = 1.0 - wBoundary*frac;

        sum += sample * w * g * wFrac;
        wsum += w * g * wFrac;
        count += g * wFrac;
      }
    }

    if (wsum > 0.0) {
      sum /= wsum;
    }

    float blend = count / sqr(2.0*radius + 1.0);
    gl_FragColor.x = mix(depth, sum, blend);
  }
  );

  const char* fragmentCompositeShader = STRINGIFY(

    uniform sampler2D tex;
  uniform vec2 invTexScale;
  uniform vec3 lightPos;
  uniform vec3 lightDir;
  uniform float spotMin;
  uniform float spotMax;
  uniform vec4 color;
  uniform float ior;

  uniform vec2 clipPosToEye;

  uniform sampler2D reflectTex;
  uniform sampler2DShadow shadowTex;
  uniform vec2 shadowTaps[12];
  uniform mat4 lightTransform;

  uniform sampler2D thicknessTex;
  uniform sampler2D sceneTex;

  uniform bool debug;

  // sample shadow map
  float shadowSample(vec3 worldPos, out float attenuation)
  {
    // hack: ENABLE_SIMPLE_FLUID
    //attenuation = 0.0f;
    //return 0.5;

    vec4 pos = lightTransform*vec4(worldPos + lightDir*0.15, 1.0);
    pos /= pos.w;
    vec3 uvw = (pos.xyz*0.5) + vec3(0.5);

    attenuation = max(smoothstep(spotMax, spotMin, dot(pos.xy, pos.xy)), 0.05);

    // user clip
    if (uvw.x  < 0.0 || uvw.x > 1.0)
      return 1.0;
    if (uvw.y < 0.0 || uvw.y > 1.0)
      return 1.0;

    float s = 0.0;
    float radius = 0.002;

    for (int i = 0; i < 8; i++)
    {
      s += shadow2D(shadowTex, vec3(uvw.xy + shadowTaps[i] * radius, uvw.z)).r;
    }

    s /= 8.0;
    return s;
  }

  vec3 viewportToEyeSpace(vec2 coord, float eyeZ)
  {
    // find position at z=1 plane
    vec2 uv = (coord*2.0 - vec2(1.0))*clipPosToEye;

    return vec3(-uv*eyeZ, eyeZ);
  }

  vec3 srgbToLinear(vec3 c) { return pow(c, vec3(2.2)); }
  vec3 linearToSrgb(vec3 c) { return pow(c, vec3(1.0 / 2.2)); }

  float sqr(float x) { return x*x; }
  float cube(float x) { return x*x*x; }

  void main()
  {
    float eyeZ = texture2D(tex, gl_TexCoord[0].xy).x;

    if (eyeZ == 0.0)
      discard;

    // reconstruct eye space pos from depth
    vec3 eyePos = viewportToEyeSpace(gl_TexCoord[0].xy, eyeZ);

    // finite difference approx for normals, can't take dFdx because
    // the one-sided difference is incorrect at shape boundaries
    vec3 zl = eyePos - viewportToEyeSpace(gl_TexCoord[0].xy - vec2(invTexScale.x, 0.0), texture2D(tex, gl_TexCoord[0].xy - vec2(invTexScale.x, 0.0)).x);
    vec3 zr = viewportToEyeSpace(gl_TexCoord[0].xy + vec2(invTexScale.x, 0.0), texture2D(tex, gl_TexCoord[0].xy + vec2(invTexScale.x, 0.0)).x) - eyePos;
    vec3 zt = viewportToEyeSpace(gl_TexCoord[0].xy + vec2(0.0, invTexScale.y), texture2D(tex, gl_TexCoord[0].xy + vec2(0.0, invTexScale.y)).x) - eyePos;
    vec3 zb = eyePos - viewportToEyeSpace(gl_TexCoord[0].xy - vec2(0.0, invTexScale.y), texture2D(tex, gl_TexCoord[0].xy - vec2(0.0, invTexScale.y)).x);

    vec3 dx = zl;
    vec3 dy = zt;

    if (abs(zr.z) < abs(zl.z))
      dx = zr;

    if (abs(zb.z) < abs(zt.z))
      dy = zb;

    //vec3 dx = dFdx(eyePos.xyz);
    //vec3 dy = dFdy(eyePos.xyz);

    vec4 worldPos = gl_ModelViewMatrixInverse*vec4(eyePos, 1.0);

    float attenuation;
    float shadow = shadowSample(worldPos.xyz, attenuation);

    vec3 l = (gl_ModelViewMatrix*vec4(lightDir, 0.0)).xyz;
    vec3 v = -normalize(eyePos);

    vec3 n = normalize(cross(dx, dy));
    vec3 h = normalize(v + l);

    vec3 skyColor = vec3(0.1, 0.2, 0.4)*1.2;
    vec3 groundColor = vec3(0.1, 0.1, 0.2);

    float fresnel = 0.1 + (1.0 - 0.1)*cube(1.0 - max(dot(n, v), 0.0));

    vec3 lVec = normalize(worldPos.xyz - lightPos);

    float ln = dot(l, n)*attenuation;

    vec3 rEye = reflect(-v, n).xyz;
    vec3 rWorld = (gl_ModelViewMatrixInverse*vec4(rEye, 0.0)).xyz;

    vec2 texScale = vec2(0.75, 1.0);	// to account for backbuffer aspect ratio (todo: pass in)

    float refractScale = ior*0.025;
    float reflectScale = ior*0.1;

    // attenuate refraction near ground (hack)
    refractScale *= smoothstep(0.1, 0.4, worldPos.y);

    vec2 refractCoord = gl_TexCoord[0].xy + n.xy*refractScale*texScale;
    //vec2 refractCoord = gl_TexCoord[0].xy + refract(-v, n, 1.0/1.33)*refractScale*texScale;	

    // read thickness from refracted coordinate otherwise we get halos around objectsw
    float thickness = max(texture2D(thicknessTex, refractCoord).x, 0.3);

    //vec3 transmission = exp(-(vec3(1.0)-color.xyz)*thickness);
    vec3 transmission = (1.0 - (1.0 - color.xyz)*thickness*0.8)*color.w;
    vec3 refract = texture2D(sceneTex, refractCoord).xyz*transmission;

    vec2 sceneReflectCoord = gl_TexCoord[0].xy - rEye.xy*texScale*reflectScale / eyePos.z;
    vec3 sceneReflect = (texture2D(sceneTex, sceneReflectCoord).xyz)*shadow;

    vec3 planarReflect = texture2D(reflectTex, gl_TexCoord[0].xy).xyz;
    planarReflect = vec3(0.0);

    // fade out planar reflections above the ground
    vec3 reflect = mix(planarReflect, sceneReflect, smoothstep(0.05, 0.3, worldPos.y)) + mix(groundColor, skyColor, smoothstep(0.15, 0.25, rWorld.y)*shadow);

    // lighting
    vec3 diffuse = color.xyz*mix(vec3(0.29, 0.379, 0.59), vec3(1.0), (ln*0.5 + 0.5)*max(shadow, 0.4))*(1.0 - color.w);
    vec3 specular = vec3(1.2*pow(max(dot(h, n), 0.0), 400.0));

    gl_FragColor.xyz = diffuse + (mix(refract, reflect, fresnel) + specular)*color.w;
    gl_FragColor.w = 1.0;

    if (debug)
      gl_FragColor = vec4(n*0.5 + vec3(0.5), 1.0);

    // write valid z
    vec4 clipPos = gl_ProjectionMatrix*vec4(0.0, 0.0, eyeZ, 1.0);
    clipPos.z /= clipPos.w;

    gl_FragDepth = clipPos.z*0.5 + 0.5;
  }
  );


  FluidRenderer* CreateFluidRenderer(uint32_t width, uint32_t height)
  {
    FluidRenderer* renderer = new FluidRenderer();

    renderer->mSceneWidth = width;
    renderer->mSceneHeight = height;

    // scene depth texture
    glVerify(glGenTextures(1, &renderer->mDepthTex));
    glVerify(glBindTexture(GL_TEXTURE_RECTANGLE_ARB, renderer->mDepthTex));

    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glVerify(glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_LUMINANCE32F_ARB, width, height, 0, GL_LUMINANCE, GL_FLOAT, NULL));

    // smoothed depth texture
    glVerify(glGenTextures(1, &renderer->mDepthSmoothTex));
    glVerify(glBindTexture(GL_TEXTURE_2D, renderer->mDepthSmoothTex));

    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE32F_ARB, width, height, 0, GL_LUMINANCE, GL_FLOAT, NULL));

    // scene copy
    glVerify(glGenTextures(1, &renderer->mSceneTex));
    glVerify(glBindTexture(GL_TEXTURE_2D, renderer->mSceneTex));

    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));

    glVerify(glGenFramebuffers(1, &renderer->mSceneFbo));
    glVerify(glBindFramebuffer(GL_FRAMEBUFFER, renderer->mSceneFbo));
    glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderer->mSceneTex, 0));

    // frame buffer
    glVerify(glGenFramebuffers(1, &renderer->mDepthFbo));
    glVerify(glBindFramebuffer(GL_FRAMEBUFFER, renderer->mDepthFbo));
    glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE_ARB, renderer->mDepthTex, 0));

    GLuint zbuffer;
    glVerify(glGenRenderbuffers(1, &zbuffer));
    glVerify(glBindRenderbuffer(GL_RENDERBUFFER, zbuffer));
    glVerify(glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height));
    glVerify(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, zbuffer));

    glVerify(glDrawBuffer(GL_COLOR_ATTACHMENT0));
    glVerify(glReadBuffer(GL_COLOR_ATTACHMENT0));

    glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo);

    // reflect texture
    glVerify(glGenTextures(1, &renderer->mReflectTex));
    glVerify(glBindTexture(GL_TEXTURE_2D, renderer->mReflectTex));

    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));

    // thickness texture
    const int thicknessWidth = width;
    const int thicknessHeight = height;

    glVerify(glGenTextures(1, &renderer->mThicknessTex));
    glVerify(glBindTexture(GL_TEXTURE_2D, renderer->mThicknessTex));

    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glVerify(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

#if USE_HDR_DIFFUSE_BLEND	
    glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, thicknessWidth, thicknessHeight, 0, GL_RGBA, GL_FLOAT, NULL));
#else
    glVerify(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, thicknessWidth, thicknessHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));
#endif

    // thickness buffer
    glVerify(glGenFramebuffers(1, &renderer->mThicknessFbo));
    glVerify(glBindFramebuffer(GL_FRAMEBUFFER, renderer->mThicknessFbo));
    glVerify(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderer->mThicknessTex, 0));

    GLuint thickz;
    glVerify(glGenRenderbuffers(1, &thickz));
    glVerify(glBindRenderbuffer(GL_RENDERBUFFER, thickz));
    glVerify(glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, thicknessWidth, thicknessHeight));
    glVerify(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, thickz));

    glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, g_msaaFbo);

    // compile shaders
    //renderer->mPointDepthProgram = CompileProgram(vertexPointDepthShader, fragmentPointDepthShader);
    renderer->mPointThicknessProgram = CompileProgram(vertexPointDepthShader, fragmentPointThicknessShader);

    //renderer->mEllipsoidThicknessProgram = CompileProgram(vertexEllipsoidDepthShader, fragmentEllipsoidThicknessShader);
    renderer->mEllipsoidDepthProgram = CompileProgram(vertexEllipsoidDepthShader, fragmentEllipsoidDepthShader, geometryEllipsoidDepthShader);

    renderer->mCompositeProgram = CompileProgram(vertexPassThroughShader, fragmentCompositeShader);
    renderer->mDepthBlurProgram = CompileProgram(vertexPassThroughShader, fragmentBlurDepthShader);

    return renderer;
  }

  void DestroyFluidRenderer(FluidRenderer* renderer)
  {
    glVerify(glDeleteFramebuffers(1, &renderer->mSceneFbo));
    glVerify(glDeleteFramebuffers(1, &renderer->mDepthFbo));
    glVerify(glDeleteTextures(1, &renderer->mDepthTex));
    glVerify(glDeleteTextures(1, &renderer->mDepthSmoothTex));
    glVerify(glDeleteTextures(1, &renderer->mSceneTex));

    glVerify(glDeleteFramebuffers(1, &renderer->mThicknessFbo));
    glVerify(glDeleteTextures(1, &renderer->mThicknessTex));
  }

  FluidRenderBuffers* CreateFluidRenderBuffers(int numFluidParticles, bool enableInterop)
  {
    FluidRenderBuffersGL* buffers = new FluidRenderBuffersGL(numFluidParticles);

    // vbos
    glVerify(glGenBuffers(1, &buffers->mPositionVBO));
    glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mPositionVBO));
    glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * numFluidParticles, 0, GL_DYNAMIC_DRAW));

    // density
    glVerify(glGenBuffers(1, &buffers->mDensityVBO));
    glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mDensityVBO));
    glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(int)*numFluidParticles, 0, GL_DYNAMIC_DRAW));

    for (int i = 0; i < 3; ++i)
    {
      glVerify(glGenBuffers(1, &buffers->mAnisotropyVBO[i]));
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mAnisotropyVBO[i]));
      glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * numFluidParticles, 0, GL_DYNAMIC_DRAW));
    }

    glVerify(glGenBuffers(1, &buffers->mIndices));
    glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers->mIndices));
    glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int)*numFluidParticles, 0, GL_DYNAMIC_DRAW));

    if (enableInterop)
    {
      buffers->mPositionBuf = NvFlexRegisterOGLBuffer(g_flexLib, buffers->mPositionVBO, numFluidParticles, sizeof(Vec4));
      buffers->mDensitiesBuf = NvFlexRegisterOGLBuffer(g_flexLib, buffers->mDensityVBO, numFluidParticles, sizeof(float));
      buffers->mIndicesBuf = NvFlexRegisterOGLBuffer(g_flexLib, buffers->mIndices, numFluidParticles, sizeof(int));

      buffers->mAnisotropyBuf[0] = NvFlexRegisterOGLBuffer(g_flexLib, buffers->mAnisotropyVBO[0], numFluidParticles, sizeof(Vec4));
      buffers->mAnisotropyBuf[1] = NvFlexRegisterOGLBuffer(g_flexLib, buffers->mAnisotropyVBO[1], numFluidParticles, sizeof(Vec4));
      buffers->mAnisotropyBuf[2] = NvFlexRegisterOGLBuffer(g_flexLib, buffers->mAnisotropyVBO[2], numFluidParticles, sizeof(Vec4));
    }

    return reinterpret_cast<FluidRenderBuffers*>(buffers);
  }

  void DestroyFluidRenderBuffers(FluidRenderBuffers* buffers)
  {
    delete reinterpret_cast<FluidRenderBuffersGL*>(buffers);
  }

  void UpdateFluidRenderBuffers(FluidRenderBuffers* buffersIn, NvFlexSolver* solver, bool anisotropy, bool density)
  {
    FluidRenderBuffersGL* buffers = reinterpret_cast<FluidRenderBuffersGL*>(buffersIn);
    // use VBO buffer wrappers to allow Flex to write directly to the OpenGL buffers
    // Flex will take care of any CUDA interop mapping/unmapping during the get() operations
    if (!anisotropy)
    {
      // regular particles
      NvFlexGetParticles(solver, buffers->mPositionBuf, NULL);
    }
    else
    {
      // fluid buffers
      NvFlexGetSmoothParticles(solver, buffers->mPositionBuf, NULL);
      NvFlexGetAnisotropy(solver, buffers->mAnisotropyBuf[0], buffers->mAnisotropyBuf[1], buffers->mAnisotropyBuf[2], NULL);
    }

    if (density)
    {
      NvFlexGetDensities(solver, buffers->mDensitiesBuf, NULL);
    }
    else
    {
      NvFlexGetPhases(solver, buffers->mDensitiesBuf, NULL);
    }

    NvFlexGetActive(solver, buffers->mIndicesBuf, NULL);
  }

  void UpdateFluidRenderBuffers(FluidRenderBuffers* buffersIn, Vec4* particles, float* densities, Vec4* anisotropy1, Vec4* anisotropy2, Vec4* anisotropy3, int numParticles, int* indices, int numIndices)
  {
    FluidRenderBuffersGL* buffers = reinterpret_cast<FluidRenderBuffersGL*>(buffersIn);
    // regular particles
    glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mPositionVBO));
    glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, buffers->mNumParticles * sizeof(Vec4), particles));

    Vec4*const anisotropies[] =
    {
      anisotropy1,
      anisotropy2,
      anisotropy3,
    };

    for (int i = 0; i < 3; i++)
    {
      Vec4* anisotropy = anisotropies[i];
      if (anisotropy)
      {
        glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mAnisotropyVBO[i]));
        glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, buffers->mNumParticles * sizeof(Vec4), anisotropy));
      }
    }

    // density /phase buffer
    if (densities)
    {
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mDensityVBO));
      glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, buffers->mNumParticles * sizeof(float), densities));
    }

    if (indices)
    {
      glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers->mIndices));
      glVerify(glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, numIndices * sizeof(int), indices));
    }

    // reset
    glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
  }

  DiffuseRenderBuffers* CreateDiffuseRenderBuffers(int numDiffuseParticles, bool& enableInterop)
  {
    DiffuseRenderBuffersGL* buffers = new DiffuseRenderBuffersGL(numDiffuseParticles);

    if (numDiffuseParticles > 0)
    {
      glVerify(glGenBuffers(1, &buffers->mDiffusePositionVBO));
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mDiffusePositionVBO));
      glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * numDiffuseParticles, 0, GL_DYNAMIC_DRAW));
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));

      glVerify(glGenBuffers(1, &buffers->mDiffuseVelocityVBO));
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mDiffuseVelocityVBO));
      glVerify(glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * numDiffuseParticles, 0, GL_DYNAMIC_DRAW));
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));

      if (enableInterop)
      {
        buffers->mDiffusePositionsBuf = NvFlexRegisterOGLBuffer(g_flexLib, buffers->mDiffusePositionVBO, numDiffuseParticles, sizeof(Vec4));
        buffers->mDiffuseVelocitiesBuf = NvFlexRegisterOGLBuffer(g_flexLib, buffers->mDiffuseVelocityVBO, numDiffuseParticles, sizeof(Vec4));
      }
    }

    return reinterpret_cast<DiffuseRenderBuffers*>(buffers);
  }

  void DestroyDiffuseRenderBuffers(DiffuseRenderBuffers* buffersIn)
  {
    DiffuseRenderBuffersGL* buffers = reinterpret_cast<DiffuseRenderBuffersGL*>(buffersIn);
    if (buffers->mNumParticles > 0)
    {
      glDeleteBuffers(1, &buffers->mDiffusePositionVBO);
      glDeleteBuffers(1, &buffers->mDiffuseVelocityVBO);

      NvFlexUnregisterOGLBuffer(buffers->mDiffusePositionsBuf);
      NvFlexUnregisterOGLBuffer(buffers->mDiffuseVelocitiesBuf);
    }
  }

  void UpdateDiffuseRenderBuffers(DiffuseRenderBuffers* buffersIn, NvFlexSolver* solver)
  {
    DiffuseRenderBuffersGL* buffers = reinterpret_cast<DiffuseRenderBuffersGL*>(buffersIn);
    // diffuse particles
    if (buffers->mNumParticles)
    {
      NvFlexGetDiffuseParticles(solver, buffers->mDiffusePositionsBuf, buffers->mDiffuseVelocitiesBuf, NULL);
    }
  }

  void UpdateDiffuseRenderBuffers(DiffuseRenderBuffers* buffersIn, Vec4* diffusePositions, Vec4* diffuseVelocities, int numDiffuseParticles)
  {
    DiffuseRenderBuffersGL* buffers = reinterpret_cast<DiffuseRenderBuffersGL*>(buffersIn);
    // diffuse particles
    if (buffers->mNumParticles)
    {
      glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mDiffusePositionVBO));
      glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, buffers->mNumParticles * sizeof(Vec4), diffusePositions));

      glVerify(glBindBuffer(GL_ARRAY_BUFFER, buffers->mDiffuseVelocityVBO));
      glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, buffers->mNumParticles * sizeof(Vec4), diffuseVelocities));
    }
  }
  
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

    glLineWidth(1.0f);

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

  // For hydrographics
  void BindHydrographicShader(Vec3 lightPos, Vec3 lightTarget, Matrix44 lightTransform, ShadowMap* shadowMap, float bias, Vec4 fogColor)
  {
    glVerify(glViewport(0, 0, g_screenWidth, g_screenHeight));

    if (s_hydrographicProgram == GLuint(-1))
    {
      s_hydrographicProgram = CompileProgram(vertexHydrographicShader, fragmentHydrographicShader);
    }

    if (s_hydrographicProgram)
    {
      glDepthMask(GL_TRUE);
      glEnable(GL_DEPTH_TEST);

      glVerify(glUseProgram(s_hydrographicProgram));

      glVerify(glUniform1i(glGetUniformLocation(s_hydrographicProgram, "grid"), 0));
      glVerify(glUniform1f(glGetUniformLocation(s_hydrographicProgram, "spotMin"), g_spotMin));
      glVerify(glUniform1f(glGetUniformLocation(s_hydrographicProgram, "spotMax"), g_spotMax));
      glVerify(glUniform4fv(glGetUniformLocation(s_hydrographicProgram, "fogColor"), 1, fogColor));

      glVerify(glUniformMatrix4fv(glGetUniformLocation(s_hydrographicProgram, "objectTransform"), 1, false, Matrix44::kIdentity));

      // set shadow parameters
      ShadowApply(s_hydrographicProgram, lightPos, lightTarget, lightTransform, shadowMap->texture);
    }
  }

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

  //glVerify(glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, g_msaaFbo));
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
    int hasTexture = showTexture && m->texCoords.size() > 0 ? 1 : 0;
    if (hasTexture) 
    {
      //Enable texture
      glVerify(glBindTexture(GL_TEXTURE_2D, m->mTextureId));
      glVerify(glEnable(GL_TEXTURE_2D));
      glVerify(glActiveTexture(GL_TEXTURE0));
      glVerify(glUniform1i(glGetUniformLocation(s_diffuseProgramV2, "tex"), 0));
      //Consider if it has a texture loaded to bind it
      glVerify(glUniform1i(glGetUniformLocation(s_diffuseProgramV2, "showTexture"), hasTexture));
    }
    //Setup normal and model mat
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_diffuseProgramV2, "normalMat"), 1, false, Transpose(AffineInverse(modelMat))));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_diffuseProgramV2, "model"), 1, false, modelMat));
    
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
  // update texture coords
  glVerify(glBindBuffer(GL_ARRAY_BUFFER, filmMesh->mPositionsVBO));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, filmMesh->mNumVertices * (sizeof(Vec4) + sizeof(Vec4)), filmMesh->mNumVertices * sizeof(Vec4), filmMesh->texCoords.data()));
  
}

void FindMeshContacts(Vec3 filmContactVertex, int filmContactVertexIndex, Vec3 filmContactPlane, GpuMesh* gpuMesh, GpuMesh* filmMesh, Mat44 modelMatrix)
{
  const float scale = 0.1f;
  for (int i = 0; i < gpuMesh->positions.size(); i++)
  {
    Vec3 positionW = modelMatrix * Vec4(gpuMesh->positions[i], 1.0f);
    float distance = Length(filmContactVertex - positionW);
    // found contact nearby a rigid mesh vertex
    if (distance <= 0.4f) 
    {
      //get texture coords from rigid mesh and transfer to soft-body
      filmMesh->texCoords[filmContactVertexIndex] = gpuMesh->texCoords[i];
      //DrawLine(filmContactVertex, filmContactVertex + Vec3(filmContactPlane)*scale, Vec4(1.0f, 0.5f, 0.0f, 0.0f));
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

void EnableShadowTexture(Texture texture)
{
  glEnable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture);
}

GpuMesh* CreateGpuMeshV2(const Mesh* m)
{
  GpuMesh* mesh = new GpuMesh();

  mesh->mNumVertices = GLuint(m->GetNumVertices());
  mesh->mNumFaces = GLuint(m->GetNumFaces());
  mesh->mNumIndices = GLuint(m->m_indices.size());

  // generate vao vbo ibo
  glVerify(glGenVertexArrays(1, &mesh->mVAO));
  glVerify(glGenBuffers(1, &mesh->mPositionsVBO));
  glVerify(glGenBuffers(1, &mesh->mIndicesIBO));

  // setup vao vbo ibo
  glVerify(glBindVertexArray(mesh->mVAO));

  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO));
  glVerify(glBufferData(GL_ARRAY_BUFFER, (sizeof(float) * 3 + sizeof(float) * 3) * mesh->mNumVertices, NULL, GL_DYNAMIC_DRAW));

  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, mesh->mNumVertices * sizeof(float) * 3, &m->m_positions[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(float) * 3, mesh->mNumVertices * sizeof(float) * 3, &m->m_normals[0]));

  glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->mIndicesIBO));
  glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * mesh->mNumIndices, &m->m_indices[0], GL_STATIC_DRAW));

  glVerify(glEnableVertexAttribArray(0));
  glVerify(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void *)0));
  glVerify(glEnableVertexAttribArray(1));
  glVerify(glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void *)(mesh->mNumVertices * sizeof(float) * 3)));

  glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
  glVerify(glBindVertexArray(0));

  return mesh;
}

GpuMesh* CreateGpuMeshTex(const Mesh* m)
{
  GpuMesh* mesh = new GpuMesh();

  mesh->mNumVertices = GLuint(m->GetNumVertices());
  mesh->mNumFaces = GLuint(m->GetNumFaces());
  mesh->mNumIndices = GLuint(m->m_indices.size());
  mesh->mTextureId = LoadTexture(m->map_Kd.c_str());

  // generate vao vbo ibo
  glVerify(glGenVertexArrays(1, &mesh->mVAO));
  glVerify(glGenBuffers(1, &mesh->mPositionsVBO));
  glVerify(glGenBuffers(1, &mesh->mIndicesIBO));

  // setup vao vbo ibo
  glVerify(glBindVertexArray(mesh->mVAO));

  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO));
  glVerify(glBufferData(GL_ARRAY_BUFFER, (sizeof(float) * 3 + sizeof(float) * 3 + sizeof(float) * 2) * mesh->mNumVertices, NULL, GL_DYNAMIC_DRAW));

  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, mesh->mNumVertices * sizeof(float) * 3, &m->m_positions[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(float) * 3, mesh->mNumVertices * sizeof(float) * 3, &m->m_normals[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * (sizeof(float) * 3 + sizeof(float) * 3), mesh->mNumVertices * sizeof(float) * 2, &m->m_texcoords[0][0]));

  glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->mIndicesIBO));
  glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * mesh->mNumIndices, &m->m_indices[0], GL_STATIC_DRAW));

  glVerify(glEnableVertexAttribArray(0));
  glVerify(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void *)0));
  glVerify(glEnableVertexAttribArray(1));
  glVerify(glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void *)(mesh->mNumVertices * sizeof(float) * 3)));
  glVerify(glEnableVertexAttribArray(2));
  glVerify(glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, (void *)(mesh->mNumVertices * (sizeof(float) * 3 + sizeof(float) * 3))));

  glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
  glVerify(glBindVertexArray(0));

  return mesh;
}

GpuMesh* CreateGpuMeshTexV2(const Mesh* m)
{
  GpuMesh* mesh = new GpuMesh();

  mesh->mNumVertices = GLuint(m->GetNumVertices());
  mesh->mNumFaces = GLuint(m->GetNumFaces());
  mesh->mNumIndices = GLuint(m->m_indices.size());
  mesh->mTextureId = LoadTexture(m->map_Kd.c_str());

  // generate vbo ibo
  glVerify(glGenBuffers(1, &mesh->mPositionsVBO));
  glVerify(glGenBuffers(1, &mesh->mNormalsVBO));
  glVerify(glGenBuffers(1, &mesh->mTexCoordsVBO));
  glVerify(glGenBuffers(1, &mesh->mIndicesIBO));

  // setup vbo ibo
  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO));
  glVerify(glBufferData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(Vec3) , &m->m_positions[0], GL_DYNAMIC_DRAW));

  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mNormalsVBO));
  glVerify(glBufferData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(Vec3), &m->m_normals[0], GL_DYNAMIC_DRAW));

  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mTexCoordsVBO));
  glVerify(glBufferData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(Vec3), &m->m_texcoords[0][0], GL_STATIC_DRAW));

  glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->mIndicesIBO));
  glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * mesh->mNumIndices, &m->m_indices[0], GL_STATIC_DRAW));

  glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
  glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

  return mesh;
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
      mesh->mTextureId = LoadTexture(texturePath.c_str());
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
      for (size_t k = 0; k < face->mNumIndices; k++) {
        int index = face->mIndices[k];
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
        nVertices++;
      }
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
  gpu_mesh->mTextureId = GLuint(mesh->mTextureId);

  std::cout << "           #Vertices   = " << gpu_mesh->mNumVertices << std::endl;
  std::cout << "           #vboVertices= " << mesh->m_positions.size() << std::endl;
  std::cout << "           #vboColors= " << mesh->m_colours.size() << std::endl;
  std::cout << "           #vboNormals= " << mesh->m_normals.size() << std::endl;
  std::cout << "           #vboTxCoords= " << mesh->m_texcoords[0].size() << std::endl;

  // same transformations when create sdf mesh
  mesh->Transform(transformation);
  mesh->Normalize(1.0f - margin);
  mesh->Transform(TranslationMatrix(Point3(margin, margin, margin)*0.5f));

  for (int i = 0; i < mesh->m_positions.size(); i++)
  {
    // can't copy directly because of type incompatibility between point and Vec3
    gpu_mesh->positions.push_back((Vec3)mesh->m_positions[i]);
    gpu_mesh->normals.push_back(mesh->m_normals[i]);

    // and also because m_texcoords is an array with dimensions [0][n];
    if (mesh->m_texcoords[0].size() == mesh->m_positions.size())
    {
      gpu_mesh->texCoords.push_back(mesh->m_texcoords[0][i]);
    }
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
  //scene = (aiScene*)aiImportFileExWithProperties(filename, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_PreTransformVertices, NULL, props); aiReleasePropertyStore(props);
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

  return gpu_mesh;
}

void SetGpuMeshTriangles(GpuMesh* gpuMesh, std::vector<Triangle> triangles) {
  gpuMesh->triangles = triangles;
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
  mesh->mTextureId = GetHydrographicTextureId(); //set texture ID for film
  mesh->texCoords.resize(0);
  for (int i = 0; i < nVertices; i++)
  {
    mesh->texCoords.push_back(Vec2(uvs[i].x, uvs[i].y));
  }

  // configure plane VAO
  glVerify(glGenVertexArrays(1, &mesh->mVAO));
  glVerify(glGenBuffers(1, &mesh->mPositionsVBO));
  glVerify(glGenBuffers(1, &mesh->mIndicesIBO));
  glVerify(glBindVertexArray(mesh->mVAO));
  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO));
  glVerify(glBufferData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec4) + sizeof(Vec4) + sizeof(Vec4)), NULL, GL_DYNAMIC_DRAW));

  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, nVertices * sizeof(Vec4), &positions[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * sizeof(Vec4), nVertices * sizeof(Vec4), &normals[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec4) + sizeof(Vec4)), nVertices * sizeof(Vec4), &uvs[0]));

  glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->mIndicesIBO));
  glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, nIndices * sizeof(int), &indices[0], GL_STATIC_DRAW));

  glVerify(glEnableVertexAttribArray(0));
  glVerify(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0));
  glVerify(glEnableVertexAttribArray(1));
  glVerify(glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(nVertices * sizeof(Vec4))));
  glVerify(glEnableVertexAttribArray(2));
  glVerify(glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(nVertices * (sizeof(Vec4) + sizeof(Vec4)))));

  glVerify(glBindVertexArray(0));

  return mesh;
}

GpuMesh* CreateGpuFilmV2(Matrix44 model, Vec4* positions, Vec4* normals, Vec4* uvs, int nVertices, int* indices, int nIndices)
{
  // buffer layout block
  // |all position vertices|all normal vertices| all tex coords|
  GpuMesh* mesh = new GpuMesh();
  mesh->mNumVertices = nVertices;
  mesh->mNumFaces = nIndices / 3;
  mesh->mNumIndices = nIndices;
  mesh->modelTransform = model;

  // configure plane VAO
  glVerify(glGenVertexArrays(1, &mesh->mVAO));
  glVerify(glGenBuffers(1, &mesh->mPositionsVBO));
  glVerify(glGenBuffers(1, &mesh->mNormalsVBO));
  glVerify(glGenBuffers(1, &mesh->mTexCoordsVBO));
  glVerify(glGenBuffers(1, &mesh->mIndicesIBO));
  glVerify(glBindVertexArray(mesh->mVAO));
  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO));
  glVerify(glBufferData(GL_ARRAY_BUFFER, nVertices * sizeof(Vec4), &positions[0], GL_STATIC_DRAW));

  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, nVertices * sizeof(Vec4), &positions[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * sizeof(Vec4), nVertices * sizeof(Vec4), &normals[0]));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, nVertices * (sizeof(Vec4) + sizeof(Vec4)), nVertices * sizeof(Vec4), &uvs[0]));

  glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->mIndicesIBO));
  glVerify(glBufferData(GL_ELEMENT_ARRAY_BUFFER, nIndices * sizeof(int), &indices[0], GL_STATIC_DRAW));

  glVerify(glEnableVertexAttribArray(0));
  glVerify(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0));
  glVerify(glEnableVertexAttribArray(1));
  glVerify(glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(nVertices * sizeof(Vec4))));
  glVerify(glEnableVertexAttribArray(2));
  glVerify(glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(nVertices * (sizeof(Vec4) + sizeof(Vec4)))));

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
  char* buf = new char[size + 2];
  memset(buf, 0, size);
  fread(buf, 1, size, fp);

  buf[size] = '\r';
  buf[size + 1] = '\n';
  fclose(fp);

  return buf;
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

// Create a GLSL program object from vertex and fragment shader files
unsigned int InitShader(const char* vShaderFile, const char* fShaderFile) {

  tShader shaders[2] = {
    { vShaderFile, GL_VERTEX_SHADER, NULL },
    { fShaderFile, GL_FRAGMENT_SHADER, NULL }
  };

  GLuint program = glCreateProgram();

  for (int i = 0; i < 2; ++i)
  {
    Shader& s = shaders[i];
    s.source = readShaderSource(s.filename);
    if (shaders[i].source == NULL)
    {
      std::cerr << "Failed to read " << s.filename << std::endl;
      exit(EXIT_FAILURE);
    }

    GLuint shader = glCreateShader(s.type);
    glShaderSource(shader, 1, (const GLchar**)&s.source, NULL);
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

    delete[] s.source;

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

GLuint GetHydrographicTextureId()
{
  return texHydrographicId;
}

GLuint GetModelTextureId()
{
  return texModelId;
}

void BindTexture(unsigned int *textureId, Vec4 *pixels, int texWidth, int texHeight)
{
	glGenTextures(1, textureId);
	glBindTexture(GL_TEXTURE_2D, *textureId);

	glTexStorage2D(GL_TEXTURE_2D, 2 /* mip map levels */, GL_RGB8, texWidth, texHeight);
	glTexSubImage2D(GL_TEXTURE_2D, 0 /* mip map level */, 0 /* xoffset */, 0 /* yoffset */, texWidth, texHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
}

void BindSolidShaderV2(Matrix44 view, Matrix44 proj, Vec3 lightPos, Vec3 camPos, Vec4 lightColor, Vec4 ambientColor, Vec4 specularColor, unsigned int specularExpoent, Vec4 diffuseColor, bool showTexture)
{
  glVerify(glViewport(0, 0, g_screenWidth, g_screenHeight));
  
  if (s_diffuseProgramV2)
  {
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    glVerify(glUseProgram(s_diffuseProgramV2));

    glVerify(glUniform3fv(glGetUniformLocation(s_diffuseProgramV2, "uLPos"), 1, lightPos));
    glVerify(glUniform4fv(glGetUniformLocation(s_diffuseProgramV2, "uLColor"), 1, lightColor));
    glVerify(glUniform4fv(glGetUniformLocation(s_diffuseProgramV2, "uColor"), 1, diffuseColor));
    glVerify(glUniform3fv(glGetUniformLocation(s_diffuseProgramV2, "uCamPos"), 1, camPos));
    glVerify(glUniform4fv(glGetUniformLocation(s_diffuseProgramV2, "uAmbient"), 1, ambientColor));
    glVerify(glUniform4fv(glGetUniformLocation(s_diffuseProgramV2, "uSpecular"), 1, specularColor));
    glVerify(glUniform1ui(glGetUniformLocation(s_diffuseProgramV2, "uSpecularExpoent"), specularExpoent));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_diffuseProgramV2, "view"), 1, false, &view[0]));
    glVerify(glUniformMatrix4fv(glGetUniformLocation(s_diffuseProgramV2, "proj"), 1, false, &proj[0]));
  }
}

void BindFilmShader(Matrix44 view, Matrix44 proj, Vec3 lightPos, Vec3 camPos, Vec4 lightColor, Vec4 ambientColor, Vec4 specularColor, unsigned int specularExpoent, Vec4 diffuseColor, bool showTexture)
{
  glVerify(glViewport(0, 0, g_screenWidth, g_screenHeight));

  if (s_filmProgram == GLuint(-1))
  {
    s_filmProgram = InitShader("../../shaders/film.vs", "../../shaders/film.fs");
  }

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



void UseSolidShader()
{
	glVerify(glUseProgram(s_diffuseProgram));
}

void UseHydrographicShader()
{
	glVerify(glUseProgram(s_hydrographicProgram));
}



void BindDisplacementsShader(Matrix44 view, Matrix44 proj, Vec3 lightPos, Vec3 lightTarget)
{
  glVerify(glUseProgram(s_displacementProgram));

  glVerify(glUniformMatrix4fv(glGetUniformLocation(s_displacementProgram, "objectTransform"), 1, false, Matrix44::kIdentity));
  glVerify(glUniformMatrix4fv(glGetUniformLocation(s_displacementProgram, "view"), 1, false, &view[0]));
  glVerify(glUniformMatrix4fv(glGetUniformLocation(s_displacementProgram, "proj"), 1, false, &proj[0]));
  glVerify(glUniformMatrix4fv(glGetUniformLocation(s_displacementProgram, "normalMat"), 1, false, Transpose(AffineInverse(Matrix44::kIdentity))));


}

void DrawDisplacements(const Vec4* positions, const Vec4* normals, const Vec4* uvs, const int numPositions, const int* indices, int numTris)
{
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  //glBufferData(GL_ARRAY_BUFFER, numPositions * (sizeof(Vec4) + sizeof(Vec4) + sizeof(Vec4)), NULL, GL_STATIC_DRAW);

  glVerify(glBufferData(GL_ARRAY_BUFFER, numPositions * sizeof(Vec4), &positions[0], GL_STATIC_DRAW));
  glVerify(glBufferData(GL_ARRAY_BUFFER, numPositions * sizeof(Vec4), &normals[0], GL_STATIC_DRAW));
  glVerify(glBufferData(GL_ARRAY_BUFFER, numPositions * sizeof(Vec4), &uvs[0], GL_STATIC_DRAW));

  glVerify(glEnableVertexAttribArray(0));
  glVerify(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vec4), (void *)0));
  glVerify(glEnableVertexAttribArray(1));
  glVerify(glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vec4), (void *)(numPositions * sizeof(Vec4))));
  glVerify(glEnableVertexAttribArray(2));
  glVerify(glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vec4), (void *)(numPositions * (sizeof(Vec4) + sizeof(Vec4)))));



  glVerify(glBindBuffer(GL_ARRAY_BUFFER, VAO));
  glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO));

  glVerify(glClientActiveTexture(GL_TEXTURE1)); //optional 

  glVerify(glDrawElements(GL_TRIANGLES, numTris, GL_UNSIGNED_INT, indices));

  glVerify(glBindVertexArray(0));
}

void UnbindHydrographicShader()
{
	glActiveTexture(GL_TEXTURE1);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);

	glUseProgram(0);
}

void EnableTexture()
{
	GLint program;
	glGetIntegerv(GL_CURRENT_PROGRAM, &program);

	if (program == GLint(s_hydrographicProgram))
	{
		glVerify(glActiveTexture(GL_TEXTURE1));//optional //0 texture unit is used by shadow pass
		glVerify(glEnable(GL_TEXTURE_2D));//optional
		glVerify(glBindTexture(GL_TEXTURE_2D, texHydrographicId));

		glVerify(glUniform1i(glGetUniformLocation(s_hydrographicProgram, "tex"), 1));//0 texture unit is used by shadow pass
		glVerify(glUniform1i(glGetUniformLocation(s_hydrographicProgram, "showTexture"), 1));
	}
}

void EnableTextureV2()
{
  if (s_filmProgram)
  {
    glVerify(glUseProgram(s_filmProgram));
    
    glVerify(glEnable(GL_TEXTURE_2D));//optional
    glVerify(glActiveTexture(GL_TEXTURE0));//optional //0 texture unit is used by shadow pass
    glVerify(glBindTexture(GL_TEXTURE_2D, texHydrographicId));

    glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "tex"), 0));//0 texture unit is used by shadow pass
    glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "showTexture"), 1));
  }
}

void EnableTexture(GLuint textureId)
{
  GLint shader;
  glGetIntegerv(GL_CURRENT_PROGRAM, &shader);
  glVerify(glUseProgram(shader));

  glVerify(glEnable(GL_TEXTURE_2D));//optional
  glVerify(glActiveTexture(GL_TEXTURE0));//optional //0 texture unit is used by shadow pass
  glVerify(glBindTexture(GL_TEXTURE_2D, textureId));

  glVerify(glUniform1i(glGetUniformLocation(shader, "tex"), 0));//0 texture unit is used by shadow pass
  glVerify(glUniform1i(glGetUniformLocation(shader, "showTexture"), 1));
}

void DisableTexture()
{
	glActiveTexture(GL_TEXTURE1);
	glDisable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glDisable(GL_TEXTURE_2D);
	glDeleteTextures(0, &texHydrographicId);
}

void DrawHydrographic(const Vec4* positions, const Vec4* normals, const float* uvs, const int* indices, int numTris, int numPositions, int colorIndex, float expand, bool twosided, bool smooth)
{

	if (!numTris)
		return;

	if (twosided)
		glDisable(GL_CULL_FACE);

#if 1
	GLint program;
	glGetIntegerv(GL_CURRENT_PROGRAM, &program);

	if (program == GLint(s_hydrographicProgram))
	{
		GLint uBias = glGetUniformLocation(s_hydrographicProgram, "bias");
		glUniform1f(uBias, 0.0f);

		GLint uExpand = glGetUniformLocation(s_hydrographicProgram, "expand");
		glUniform1f(uExpand, expand);

	}
#endif

	glColor3fv(g_colors[colorIndex] * 1.5f);
	glSecondaryColor3fv(g_colors[colorIndex] * 1.5f);

	glVerify(glBindBuffer(GL_ARRAY_BUFFER, 0));
	glVerify(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

	glVerify(glEnableClientState(GL_VERTEX_ARRAY));
	glVerify(glEnableClientState(GL_NORMAL_ARRAY));
	//enable texture
	glVerify(glClientActiveTexture(GL_TEXTURE1)); //optional 
	glVerify(glEnableClientState(GL_TEXTURE_COORD_ARRAY));

	glVerify(glVertexPointer(3, GL_FLOAT, sizeof(float) * 4, positions));
	glVerify(glNormalPointer(GL_FLOAT, sizeof(float) * 4, normals));
	glVerify(glTexCoordPointer(3, GL_FLOAT, sizeof(float) * 3, uvs));

	glVerify(glDrawElements(GL_TRIANGLES, numTris * 3, GL_UNSIGNED_INT, indices));

	glVerify(glDisableClientState(GL_VERTEX_ARRAY));
	glVerify(glDisableClientState(GL_NORMAL_ARRAY));
	glVerify(glDisableClientState(GL_TEXTURE_COORD_ARRAY));


	if (twosided)
		glEnable(GL_CULL_FACE);

#if 1
	if (program == GLint(s_hydrographicProgram))
	{
		GLint uBias = glGetUniformLocation(s_hydrographicProgram, "bias");
		glUniform1f(uBias, g_shadowBias);

		GLint uExpand = glGetUniformLocation(s_hydrographicProgram, "expand");
		glUniform1f(uExpand, 0.0f);
	}
#endif

}

void DrawHydrographicV2(GpuMesh* mesh, const Vec4* positions, const Vec4* normals, const Vec4* uvs, const int* indices, int nIndices, int numPositions, bool showTexture)
{
  // Enable texture
  glVerify(glEnable(GL_TEXTURE_2D));//
  glVerify(glActiveTexture(GL_TEXTURE0));//
  glVerify(glBindTexture(GL_TEXTURE_2D, mesh->mTextureId));
  glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "tex"), 0));//
  int hasTexture = showTexture && mesh->texCoords.size() > 0 ? 1 : 0;
  glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "showTexture"), hasTexture));
  // update positions and normals
  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO)); // 
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, mesh->mNumVertices * sizeof(Vec4), positions));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(Vec4), mesh->mNumVertices * sizeof(Vec4), normals));
  // draw VAO
  glVerify(glBindVertexArray(mesh->mVAO));
  glVerify(glDrawElements(GL_TRIANGLES, nIndices, GL_UNSIGNED_INT, 0));
  glVerify(glBindVertexArray(0));
  // disable texture
  glActiveTexture(GL_TEXTURE0);
  glDisable(GL_TEXTURE_2D);
}

void DrawDistortion(GpuMesh* mesh, const Vec4* positions, const Vec4* normals, const Vec4* uvs, const int* indices, int nIndices, int numPositions, bool showTexture)
{
  // Enable texture
  glVerify(glEnable(GL_TEXTURE_2D));//
  glVerify(glActiveTexture(GL_TEXTURE0));//
  glVerify(glBindTexture(GL_TEXTURE_2D, mesh->mTextureId));
  glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "tex"), 0));//
  int hasTexture = showTexture && mesh->texCoords.size() ? 1 : 0;
  glVerify(glUniform1i(glGetUniformLocation(s_filmProgram, "showTexture"), hasTexture));
  // update positions and normals
  glVerify(glBindBuffer(GL_ARRAY_BUFFER, mesh->mPositionsVBO)); // parei aqui -> criar condicao para selecionar textura diferente, caso seja a transferencia do objeto para o filme
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, 0, mesh->mNumVertices * sizeof(Vec4), positions));
  glVerify(glBufferSubData(GL_ARRAY_BUFFER, mesh->mNumVertices * sizeof(Vec4), mesh->mNumVertices * sizeof(Vec4), normals));
  // draw VAO
  glVerify(glBindVertexArray(mesh->mVAO));
  glVerify(glDrawElements(GL_TRIANGLES, nIndices, GL_UNSIGNED_INT, 0));
  glVerify(glBindVertexArray(0));
  // disable texture
  glActiveTexture(GL_TEXTURE0);
  glDisable(GL_TEXTURE_2D);
}
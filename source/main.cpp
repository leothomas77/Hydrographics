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
// Copyright (c) 2013-2017 NVIDIA Corporation. All rights reserved.

#include "../core/types.h"
#include "../core/maths.h"
#include "../core/platform.h"
#include "../core/mesh.h"
#include "../core/voxelize.h"
#include "../core/sdf.h"
#include "../core/pfm.h"
#include "../core/tga.h"
#include "../core/perlin.h"
#include "../core/convex.h"
#include "../core/cloth.h"

#include "../external/SDL2-2.0.4/include/SDL.h"

#include "../include/NvFlex.h"
#include "../include/NvFlexExt.h"
#include "../include/NvFlexDevice.h"

#include <iostream>
#include <map>
#include <cuda_runtime.h>

#include "shaders.h"
#include "imgui.h"
#include "shadersDemoContext.h"
#include "ColorGradient.h"

#include "model_data.h"
#include "main.h"
#include "helpers.h"
#include "../source/scenes.h"
#include "benchmark.h"

using namespace std;

void Init(int scene, bool centerCamera = true, bool resetSimParams = true)
{
	RandInit();

	if (g_solver)
	{
    Destroy();
	}

	// alloc buffers
	g_buffers = AllocBuffers(g_flexLib);

	// map during initialization
	MapBuffers(g_buffers);

	g_buffers->positions.resize(0);
  g_buffers->velocities.resize(0);
	g_buffers->phases.resize(0);
  g_buffers->normals.resize(0);
  g_buffers->activeIndices.resize(0);
  
	g_buffers->springIndices.resize(0);
	g_buffers->springLengths.resize(0);
	g_buffers->springStiffness.resize(0);

	g_buffers->triangles.resize(0);
	g_buffers->triangleNormals.resize(0);
	g_buffers->uvs.resize(0);

  ClearShapes();

  // for draw distortions
  g_contact_positions.resize(0);
  g_contact_normals.resize(0);
  g_contact_uvs.resize(0);
  g_contact_indexes.resize(0);
  // for draw color compensation
  g_stretch_colors.resize(0);
  g_compens_colors.resize(0);

  // for compute fps and the metrics
  g_avgFPS = g_avgUpdateTime = g_avgRenderTime = g_avgWaitTime = g_avgLatencyTime = 0.0;
  g_fps = g_updateTime = g_renderTime = g_waitTime = g_simLatency = 0.0f;
  frametimes.resize(0);
  update_times.resize(0);
  render_times.resize(0);
  wait_times.resize(0);
  latency_times.resize(0);

	g_frame = 0;
	g_pause = false;

  g_dt = 1.0f / 60.0f;

	g_lightDistance = 2.0f;

  g_camPos.resize(0);
	g_camSpeed = 0.05f;
  
	// sim params
  if (resetSimParams)
  {
    g_params.gravity[0] = 0.0f;
    g_params.gravity[1] = 0.0f;
    g_params.gravity[2] = 0.0f;

    g_params.wind[0] = 0.0f;
    g_params.wind[1] = 0.0f;
    g_params.wind[2] = 0.0f;

    g_params.radius = 0.15f;
    g_params.viscosity = 0.0f;
    g_params.dynamicFriction = 0.0f;
    g_params.staticFriction = 0.0f;
    g_params.particleFriction = 0.0f; // scale friction between particles by default
    g_params.freeSurfaceDrag = 0.0f;
    g_params.drag = 0.0f;
    g_params.lift = 0.0f;
    g_params.numIterations = 3;
    g_numSubsteps = 2;
    g_params.fluidRestDistance = 0.0f;
    g_params.solidRestDistance = 0.0f;

    g_params.anisotropyScale = 1.0f;
    g_params.anisotropyMin = 0.1f;
    g_params.anisotropyMax = 2.0f;
    g_params.smoothing = 1.0f;

    g_params.dissipation = 0.0f;
    g_params.damping = 0.0f;
    g_params.particleCollisionMargin = 0.0f;
    g_params.shapeCollisionMargin = 0.0f;
    g_params.collisionDistance = 0.0f;
    g_params.sleepThreshold = 0.0f;
    g_params.shockPropagation = 0.0f;
    g_params.restitution = 0.0f;

    g_params.maxSpeed = FLT_MAX;
    g_params.maxAcceleration = 100.0f;	// approximately 10x gravity

    g_params.relaxationMode = eNvFlexRelaxationLocal;
    g_params.relaxationFactor = 1.0f;
    g_params.solidPressure = 1.0f;
    g_params.adhesion = 0.0f;
    g_params.cohesion = 0.025f;
    g_params.surfaceTension = 0.0f;
    g_params.vorticityConfinement = 0.0f;
    g_params.buoyancy = 1.0f;
    g_params.diffuseThreshold = 100.0f;
    g_params.diffuseBuoyancy = 1.0f;
    g_params.diffuseDrag = 0.8f;
    g_params.diffuseBallistic = 16;
    g_params.diffuseLifetime = 2.0f;


    // planes created after particles
    g_params.numPlanes = 1;

  }

  g_warmup = false;

	g_maxDiffuseParticles = 0;	// number of diffuse particles
	g_maxNeighborsPerParticle = 96;
	g_numExtraParticles = 0;	// number of particles allocated but not made active	
	g_maxContactsPerParticle = 6;

	g_sceneLower = FLT_MAX;
	g_sceneUpper = -FLT_MAX;

	// initialize solver desc
	NvFlexSetSolverDescDefaults(&g_solverDesc);

	// create scene
	StartGpuWork();
	g_scenes[g_scene]->Initialize(resetSimParams);
	EndGpuWork();

	uint32_t numParticles = g_buffers->positions.size();
	uint32_t maxParticles = numParticles + g_numExtraParticles*g_numExtraMultiplier;

	if (g_params.solidRestDistance == 0.0f)
	g_params.solidRestDistance = g_params.radius;

	// if fluid present then we assume solid particles have the same radius
	if (g_params.fluidRestDistance > 0.0f)
		g_params.solidRestDistance = g_params.fluidRestDistance;

	// set collision distance automatically based on rest distance if not alraedy set
	if (g_params.collisionDistance == 0.0f)
		g_params.collisionDistance = Max(g_params.solidRestDistance, g_params.fluidRestDistance)*0.5f;

	// default particle friction to 10% of shape friction
	if (g_params.particleFriction == 0.0f)
		g_params.particleFriction = g_params.dynamicFriction*0.1f;

	// add a margin for detecting contacts between particles and shapes
	if (g_params.shapeCollisionMargin == 0.0f)
		g_params.shapeCollisionMargin = g_params.collisionDistance*0.5f;

	// calculate particle bounds
	Vec3 particleLower, particleUpper;
	GetParticleBounds(particleLower, particleUpper);

	// accommodate shapes
	Vec3 shapeLower, shapeUpper;
	GetShapeBounds(shapeLower, shapeUpper);

	// update bounds
	g_sceneLower = Min(Min(g_sceneLower, particleLower), shapeLower);
	g_sceneUpper = Max(Max(g_sceneUpper, particleUpper), shapeUpper);

	g_sceneLower -= g_params.collisionDistance;
	g_sceneUpper += g_params.collisionDistance;

  Vec3 perspectiveCamPosition = Vec3((g_sceneLower.x + g_sceneUpper.x)*0.5f, min(g_sceneUpper.y*1.25f, 6.0f), g_sceneUpper.z + min(g_sceneUpper.y, 6.0f));
  //main parspective camera
  g_camPos.push_back(perspectiveCamPosition);
  g_camAngle.push_back(Vec3(0.0f, DegToRad(-30.0f), 0.0f));
  //bottom camera
  g_camPos.push_back(Vec3(g_meshCenter.x, -2.4f, g_meshCenter.z));
  g_camAngle.push_back(Vec3(0.0f, DegToRad(90.0f), 0.0f));
  //top camera
  g_camPos.push_back(Vec3(0.0f, 0.83f, 0.0f));
  g_camAngle.push_back(Vec3(DegToRad(90.0f), DegToRad(-90.0f), 0.0f));

  g_camIndex = 0; // select main cam view

	g_solverDesc.maxParticles = maxParticles;
	g_solverDesc.maxDiffuseParticles = g_maxDiffuseParticles;
	g_solverDesc.maxNeighborsPerParticle = g_maxNeighborsPerParticle;
	g_solverDesc.maxContactsPerParticle = g_maxContactsPerParticle;

	// main create method for the Flex solver
	g_solver = NvFlexCreateSolver(g_flexLib, &g_solverDesc);

	// give scene a chance to do some post solver initialization
	g_scenes[g_scene]->PostInitialize();

	if (centerCamera)
	{
    // reset the perspective cam
    g_camIndex = 0;
		g_camPos[g_camIndex] = Vec3((g_sceneLower.x + g_sceneUpper.x)*0.5f, min(g_sceneUpper.y*1.25f, 6.0f), g_sceneUpper.z + min(g_sceneUpper.y, 6.0f));
		g_camAngle[g_camIndex] = Vec3(0.0f, DegToRad(-30.0f), 0.0f);
		// give scene a chance to modify camera position
		g_scenes[g_scene]->CenterCamera();
	}

	// create active indices (just a contiguous block for the demo)
  g_buffers->activeIndices.resize(g_buffers->positions.size());

  for (int i = 0; i < g_buffers->activeIndices.size(); ++i)
  {
		g_buffers->activeIndices[i] = i;
  }

	// resize particle buffers to fit
	g_buffers->positions.resize(maxParticles);
	g_buffers->velocities.resize(maxParticles);
	g_buffers->phases.resize(maxParticles);

	// save rest positions
	g_buffers->restPositions.resize(g_buffers->positions.size());
  for (int i = 0; i < g_buffers->positions.size(); ++i)
  {
		g_buffers->restPositions[i] = g_buffers->positions[i];
  }

	// unmap so we can start transferring data to GPU
	UnmapBuffers(g_buffers);

  //-----------------------------
	// Send data to Flex

	NvFlexCopyDesc copyDesc;
	copyDesc.dstOffset = 0;
	copyDesc.srcOffset = 0;
	copyDesc.elementCount = numParticles;

	NvFlexSetParams(g_solver, &g_params);
	NvFlexSetParticles(g_solver, g_buffers->positions.buffer, &copyDesc);
	NvFlexSetVelocities(g_solver, g_buffers->velocities.buffer, &copyDesc);
	NvFlexSetNormals(g_solver, g_buffers->normals.buffer, &copyDesc);
	NvFlexSetPhases(g_solver, g_buffers->phases.buffer, &copyDesc);
	NvFlexSetRestParticles(g_solver, g_buffers->restPositions.buffer, &copyDesc);

	NvFlexSetActive(g_solver, g_buffers->activeIndices.buffer, &copyDesc);
	NvFlexSetActiveCount(g_solver, numParticles);

	// springs
	if (g_buffers->springIndices.size())
	{
		assert((g_buffers->springIndices.size() & 1) == 0);
		assert((g_buffers->springIndices.size() / 2) == g_buffers->springLengths.size());

		NvFlexSetSprings(g_solver, g_buffers->springIndices.buffer, g_buffers->springLengths.buffer, g_buffers->springStiffness.buffer, g_buffers->springLengths.size());
  }

	// dynamic triangles
	if (g_buffers->triangles.size())
	{
		NvFlexSetDynamicTriangles(g_solver, g_buffers->triangles.buffer, g_buffers->triangleNormals.buffer, g_buffers->triangles.size() / 3);
	}

	// collision shapes
	if (g_buffers->shapeFlags.size())
	{
		NvFlexSetShapes(
			g_solver,
			g_buffers->shapeGeometry.buffer,
			g_buffers->shapePositions.buffer,
			g_buffers->shapeRotations.buffer,
			g_buffers->shapePrevPositions.buffer,
			g_buffers->shapePrevRotations.buffer,
			g_buffers->shapeFlags.buffer,
			int(g_buffers->shapeFlags.size()));
	}

	// perform initial sim warm up
	if (g_warmup)
	{
		printf("Warming up sim..\n");

		// warm it up (relax positions to reach rest density without affecting velocity)
		NvFlexParams copy = g_params;
		copy.numIterations = 4;

		NvFlexSetParams(g_solver, &copy);

		const int kWarmupIterations = 100;

		for (int i = 0; i < kWarmupIterations; ++i)
		{
			NvFlexUpdateSolver(g_solver, 0.0001f, 1, false);
			NvFlexSetVelocities(g_solver, g_buffers->velocities.buffer, NULL);
		}

		// udpate host copy
		NvFlexGetParticles(g_solver, g_buffers->positions.buffer, NULL);
	
		printf("Finished warm up.\n");
	}
}

void Reset()
{
	Init(g_scene, true, g_resetSimParams);
  g_resetSimParams = true;
}

void Destroy()
{
  // free buffers
  DestroyBuffers(g_buffers);

  for (auto& iter : g_meshes)
  {
    NvFlexDestroyTriangleMesh(g_flexLib, iter.first);
    DestroyGpuMesh(iter.second);
  }

  int sdfCount = 0;
  NvFlexGetDistanceFields(g_flexLib, &g_sdf_mesh, sdfCount);
  if (sdfCount != 0)
  {
    printf("Destroying existing SDF");
    NvFlexDestroyDistanceField(g_flexLib, g_sdf_mesh);
  }

  DestroyGpuMesh(g_gpu_rigid_mesh);

  DestroyGpuMesh(g_gpu_film_mesh);

  g_meshes.clear();

  NvFlexDestroySolver(g_solver);

}

void Shutdown()
{
  Destroy();

  NvFlexShutdown(g_flexLib);

#if _WIN32
	if (g_ffmpeg)
		_pclose(g_ffmpeg);
#endif
}

void UpdateCamera()
{
	Vec3 forward(-sinf(g_camAngle[g_camIndex].x)*cosf(g_camAngle[g_camIndex].y), sinf(g_camAngle[g_camIndex].y), -cosf(g_camAngle[g_camIndex].x)*cosf(g_camAngle[g_camIndex].y));
	Vec3 right(Normalize(Cross(forward, Vec3(0.0f, 1.0f, 0.0f))));

	g_camSmoothVel = Lerp(g_camSmoothVel, g_camVel, 0.1f);
	g_camPos[g_camIndex] += (forward*g_camSmoothVel.z + right*g_camSmoothVel.x + Cross(right, forward)*g_camSmoothVel.y);
}

void SyncScene()
{
	// let the scene send updates to flex directly
	g_scenes[g_scene]->Sync();
}

void UpdateScene()
{
	// give scene a chance to make changes to particle buffers
	g_scenes[g_scene]->Update();
}

void RenderScene()
{


  //Mat44 projOrtho = OrthographicMatrix(0.0f, float(g_screenWidth), 0.0f, float(g_screenHeight), g_camNear, g_camFar);
  //float max = MAX(g_sceneUpper.x, g_sceneUpper.y);
  //float r = max * g_aspect, t = max;
  //float l = -r, b = -t;
  //Mat44 projOrtho = OrthographicMatrix(l, r, b, t, g_camNear, g_camFar);
  //float aspect = (g_screenWidth*fy) / (g_screenHeight*fx);
  //g_proj = projOrtho;
  //g_proj = ProjectionMatrix(RadToDeg(g_fov), g_aspect, g_camNear, g_camFar);

  g_proj = Frustum(-g_screenWidth, g_screenWidth, -g_screenHeight, g_screenHeight, g_camNear, g_camFar);
  g_camDistance = Length(g_camPos[g_camIndex] - g_centroid);
 
  g_view = RotationMatrix(-g_camAngle[g_camIndex].x, Vec3(0.0f, 1.0f, 0.0f))*RotationMatrix(-g_camAngle[g_camIndex].y, Vec3(cosf(-g_camAngle[g_camIndex].x), 0.0f, sinf(-g_camAngle[g_camIndex].x)))*TranslationMatrix(-Point3(g_camPos[g_camIndex]));

  g_lightPos = g_camPos[g_camIndex]; // cam target always iluminated
  bool showTexture = true;
  
  SetViewport(0, 0, g_screenWidth, g_screenHeight);

  BindRigidBodyShader(g_view, g_proj, g_lightPos, g_camPos[g_camIndex], g_lightColor, g_ambientColor, g_specularColor, g_specularExpoent, g_diffuseColor);

  SetCullMode(false);
  SetFillMode(g_wireframe);

  if (g_drawHydrographicCollisionMesh) // draw rigid
  {
    DrawGpuMeshV2(g_gpu_rigid_mesh, g_model, showTexture);
  }

  if (g_drawHydrographic) // draw film
  {
    showTexture = true;
    BindFilmShader(g_view, g_proj, g_lightPos, g_camPos[g_camIndex], g_lightColor, g_ambientColor, g_specularColor, g_specularExpoent, g_diffuseColor);
    DrawHydrographicFilm(g_gpu_film_mesh, &g_buffers->positions[0], &g_buffers->normals[0], &g_buffers->uvs[0], &g_buffers->triangles[0], g_buffers->triangles.size(), g_buffers->positions.size(), showTexture);
  }
  
  // draw reverse texure
  if (!isTestingMode() && !g_generateContactsTexture)
  {
    if (g_createReverseTextureFile)
    {
      g_createReverseTextureFile = false;//run this only once
      g_camIndexAux = g_camIndex;//save current view
      g_camIndex = 2;
      Vec3 camPos = g_camPos[g_camIndex];
      camPos.y = CAM_DISTANCE_R * g_realDistanceFactor;
      g_proj = Frustum(-IMG_WIDTH, IMG_WIDTH, -g_screenHeight, g_screenHeight, g_camNear, g_camFar);
      g_view = RotationMatrix(-g_camAngle[g_camIndex].x, Vec3(0.0f, 1.0f, 0.0f))*RotationMatrix(-g_camAngle[g_camIndex].y, Vec3(cosf(-g_camAngle[g_camIndex].x), 0.0f, sinf(-g_camAngle[g_camIndex].x)))*TranslationMatrix(-Point3(camPos));
      BindReverseTextureShader(g_view, g_proj, g_lightPos, camPos, g_lightColor, g_ambientColor, g_specularColor, g_specularExpoent, g_diffuseColor, g_max_distance_uv, g_near_distance_uv, g_weight1, g_weight2, g_tesselation_inner, g_tesselation_outer);
      DrawReverseTexture(g_gpu_film_mesh, &g_contact_positions[0], &g_contact_normals[0], &g_contact_uvs[0], &g_contact_indexes[0], g_contact_indexes.size(), g_contact_positions.size(), &g_compens_colors[0], g_textureMode);
      CreateHydrographicFilmImage(g_screenWidth, g_screenHeight, IMG_WIDTH);
      g_camIndex = g_camIndexAux;//restore current view
    }
    else
    {
      if (g_drawReverseTexture || g_drawStretchColor || g_drawColorCompensation) {
        //render the reverse texture in the selected mode
        BindReverseTextureShader(g_view, g_proj, g_lightPos, g_camPos[g_camIndex], g_lightColor, g_ambientColor, g_specularColor, g_specularExpoent, g_diffuseColor, g_max_distance_uv, g_near_distance_uv, g_weight1, g_weight2, g_tesselation_inner, g_tesselation_outer);
        DrawReverseTexture(g_gpu_film_mesh, &g_contact_positions[0], &g_contact_normals[0], &g_contact_uvs[0], &g_contact_indexes[0], g_contact_indexes.size(), g_contact_positions.size(), &g_compens_colors[0], g_textureMode);
      }
    }
  }
  
}


void RenderDebug()
{

  SetView(g_view, g_proj);
  SetCullMode(true);

  // springs
	if (g_drawSprings)
	{
		Vec4 color;

		if (g_drawSprings == 1)
		{
			// stretch 
			color = Vec4(0.0f, 0.0f, 1.0f, 0.8f);
		}
		if (g_drawSprings == 2)
		{
			// tether
			color = Vec4(0.0f, 1.0f, 0.0f, 0.8f);
		}

		//BeginLines();
    BeginPoints(3.0f);
		int start = 0;

		ColorGradient *colorGradient = new ColorGradient();

    float maxDisplacement = 0.0f;

		for (int i = start; i < g_buffers->springLengths.size(); ++i)
		{
			if (g_drawSprings == 1 && g_buffers->springStiffness[i] < 0.0f)
				continue;
			if (g_drawSprings == 2 && g_buffers->springStiffness[i] > 0.0f)
				continue;

			int a = g_buffers->springIndices[i * 2];
			int b = g_buffers->springIndices[i * 2 + 1];
			
      //Heatmap of distortions
      if (g_drawStretching)
      {
        g_drawStiffness = false;

        float distortedLength = Length(g_buffers->positions[a] - g_buffers->positions[b]);
        Vec3 restA = g_contact_positions[a];
        Vec3 restB = g_contact_positions[b];
        float restLength = Length(restA - restB);
        float displacementCoef = abs(distortedLength - restLength) / restLength;
        //color = colorGradient->getColorAtValue(displacementCoef * g_stretchFactor);
      }

      //Heatmap of stiffness
      if (g_drawStiffness)
      {
        color = colorGradient->getColorAtValue(g_buffers->springStiffness[i]);
      }

			//DrawLine(Vec3(g_contact_positions[a]), Vec3(g_contact_positions[b]), color);
      DrawPoint(Vec3(g_contact_positions[a]), color);
      DrawPoint(Vec3(g_contact_positions[b]), color);
    }

		//EndLines();
    EndPoints();
	}
	// visualize contacts against the environment
  // all arrays are iterated in CPU
	if (g_drawContacts)
	{
		const int maxContactsPerParticle = 6;
		NvFlexVector<Vec4> contactPlanes(g_flexLib, g_buffers->positions.size()*maxContactsPerParticle);
		NvFlexVector<Vec4> contactVelocities(g_flexLib, g_buffers->positions.size()*maxContactsPerParticle);
		NvFlexVector<int> contactIndices(g_flexLib, g_buffers->positions.size());
		NvFlexVector<unsigned int> contactCounts(g_flexLib, g_buffers->positions.size());

		NvFlexGetContacts(g_solver, contactPlanes.buffer, contactVelocities.buffer, contactIndices.buffer, contactCounts.buffer);

		// ensure transfers have finished
		contactPlanes.map();
		contactVelocities.map();
		contactIndices.map();
		contactCounts.map();

		// for each active particle of simulation
		for (int i = 0; i < int(g_buffers->activeIndices.size()); ++i)
		{
			// each active particle can have up to 6 contact points on NVIDIA Flex 1.1.0
      const int filmIndex = g_buffers->activeIndices[i];
      Vec3 filmNormal = g_buffers->normals[filmIndex];
      Vec3 filmPosition = Vec3(g_buffers->positions[filmIndex]);
			const int contactIndex = contactIndices[filmIndex];
			const unsigned int count = contactCounts[contactIndex];
			//const float scale = 0.1f;

      //if an active particle have entries in the contact counts array, this particle have contacts
      if (count)
      {
			  //retrieve contact planes for each particle 
        // in pratice, the contact plane shows better smoothed reverse texture mapping results
			  for (unsigned int c = 0; c < count; ++c)
			  {
				  Vec4 filmContactPlane = contactPlanes[contactIndex*maxContactsPerParticle + c];
          BuildContactUVs(filmPosition, filmIndex, -Vec3(filmContactPlane), g_gpu_rigid_mesh, g_model, g_contact_positions, g_contact_uvs);
			  }

      }

		}

    //FixTextureSeams(g_film_mesh, g_contact_positions, g_contact_uvs);
	
  }

  if (g_drawNeighbors)
  {
    NvFlexVector<int> neighbors(g_flexLib, g_solverDesc.maxParticles * g_solverDesc.maxNeighborsPerParticle);
    NvFlexVector<int> neighborCounts(g_flexLib, g_solverDesc.maxParticles);
    NvFlexVector<int> apiToInternal(g_flexLib, g_solverDesc.maxParticles);
    NvFlexVector<int> internalToApi(g_flexLib, g_solverDesc.maxParticles);

    NvFlexGetNeighbors(g_solver, neighbors.buffer, neighborCounts.buffer, apiToInternal.buffer, internalToApi.buffer);
    // neighbors are stored in a strided format so that the first neighbor
    // of each particle is stored sequentially, then the second, and so on

    neighbors.map();
    neighborCounts.map();
    apiToInternal.map();
    internalToApi.map();

    int stride = g_solverDesc.maxParticles;

    BeginLines();
    Vec4 redColor = Vec4(1.0f, 0.0f, 0.0f, 1.0f);
    Vec4 yellowColor = Vec4(1.0f, 1.0f, 0.0f, 1.0f);
    //DrawLine(Vec3(0.0f), Vec3(0.0f) + Vec3(0.0f, 1.0f, 0.0f)* 0.1f ,yellowColor);


    for (int i = 0; i < int(g_buffers->positions.size()); ++i)
    //int i = 8;
    {
      // find offset in the neighbors buffer
      int offset = apiToInternal[i];
      int count = neighborCounts[offset];

      for (int c = 0; c < count; ++c)
      {
        int neighbor = internalToApi[neighbors[c*stride + offset]];

        Vec3 particle = Vec3(g_buffers->positions[i]);
        //Vec3 normal = Vec3(g_buffers->normals[i]);

        Vec3 particleNeighbor = Vec3(g_buffers->positions[neighbor]);
        //Vec3 neighborNormal = Vec3(g_buffers->normals[neighbor]);

        DrawLine(particle, particleNeighbor, redColor);

        //DrawLine(particleNeighbor, particleNeighbor + normal * 0.05f, yellowColor);

        //printf("Particle %d's neighbor %d is particle %d\n", i, c, neighbor);
      }
    }
    neighbors.destroy();
    neighborCounts.destroy();
    apiToInternal.destroy();
    internalToApi.destroy();

    EndLines();
  }

	if (g_drawAxis)
	{
    
    BeginLines();

		float size = 0.5f;

		Vec3 red   = {1.0f, 0.0f, 0.0f};
		Vec3 green = {0.0f, 1.0f, 0.0f};
		Vec3 blue  = {0.0f, 0.0f, 1.0f};

		DrawLine(Vec3(0.0f, 0.0f, 0.0f), Vec3(size, 0.0f, 0.0f), Vec4(red, 0.0f));
		DrawLine(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, size, 0.0f), Vec4(green, 0.0f));
		DrawLine(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, size), Vec4(blue, 0.0f));

		EndLines();

    BeginPoints(3.0f);
    DrawPoint(g_centroid, Vec4(red, 0.0f));
    EndPoints();


  }

	if (g_drawNormals)
	{
		NvFlexGetNormals(g_solver, g_buffers->normals.buffer, NULL);
    float scale = .2f;
		BeginLines();

		for (int i = 0; i < g_buffers->normals.size(); ++i)
		{
			DrawLine(Vec3(g_buffers->positions[i]),
				Vec3(g_buffers->positions[i] - g_buffers->normals[i] * g_buffers->normals[i].w),
				Vec4(0.0f, 1.0f, 0.0f, 0.0f));
		}

		EndLines();
	}
}
// returns the new scene if one is selected
int DoUI()
{
	// gui may set a new scene
	int newScene = -1;

	if (g_showHelp)
	{
		const int numParticles = NvFlexGetActiveCount(g_solver);
		// const int numDiffuse = g_buffers->diffuseCount[0];

		int x = g_screenWidth - 200;
		int y = g_screenHeight - 23;

		// imgui
		unsigned char button = 0;
		if (g_lastb == SDL_BUTTON_LEFT)
			button = IMGUI_MBUT_LEFT;
		else if (g_lastb == SDL_BUTTON_RIGHT)
			button = IMGUI_MBUT_RIGHT;

		imguiBeginFrame(g_lastx, g_screenHeight - g_lasty, button, 0);

		x += 180;

		int fontHeight = 13;

		if (1)
		{
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Frame: %d", g_frame); y -= fontHeight * 2;

			if (!g_ffmpeg)
			{
				DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "FPS: %.2f", g_fps); y -= fontHeight * 2;
        if (isTestingMode())
        {
          DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "AVG FPS: %.2f", g_avgFPS); y -= fontHeight * 2;
          DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "AVG Update Time: %.2fms", g_avgUpdateTime*1000.0f); y -= fontHeight * 2;
          DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "AVG Render Time: %.2fms", g_avgRenderTime*1000.0f); y -= fontHeight * 2;
          DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "AVG Wait Time: %.2fms", g_avgWaitTime*1000.0f); y -= fontHeight * 2;
        }
        DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Frame Time: %.2fms", g_realdt*1000.0f); y -= fontHeight * 2;

				// If detailed profiling is enabled, then these timers will contain the overhead of the detail timers, so we won't display them.
				if (!g_profile)
				{
          DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Time (CPU): %.2fms", g_updateTime*1000.0f); y -= fontHeight;
          DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Render Time (CPU): %.2fms", g_renderTime*1000.0f); y -= fontHeight;
          DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Wait Time (CPU): %.2fms", g_waitTime*1000.0f); y -= fontHeight;
					DrawImguiString(x, y, Vec3(0.97f, 0.59f, 0.27f), IMGUI_ALIGN_RIGHT, "Sim Latency (GPU): %.2fms", g_simLatency); y -= fontHeight * 2;
				}
				else
				{
					y -= fontHeight * 3;
				}
			}

			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Particle Count: %d", numParticles); y -= fontHeight;
			//DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Diffuse Count: %d", numDiffuse); y -= fontHeight;
			//DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Rigid Count: %d", g_buffers->rigidOffsets.size() > 0 ? g_buffers->rigidOffsets.size() - 1 : 0); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Spring Count: %d", g_buffers->springLengths.size()); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Num Substeps: %d", g_numSubsteps); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Num Iterations: %d", g_params.numIterations); y -= fontHeight * 2;

			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Device: %s", g_deviceName); y -= fontHeight * 2;
      DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Camera Positon (X ,Y ,Z): (%.2f, %.2f, %.2f)", g_camPos[g_camIndex].x, g_camPos[g_camIndex].y, g_camPos[g_camIndex].z); y -= fontHeight * 2;
      DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Cam distance %.2f", g_camDistance); y -= fontHeight * 2;
      DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Cam real distance %.2f mm", g_camDistance / g_realDistanceFactor); y -= fontHeight * 2;
      //DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Real dipping %.2f mm/s", g_realDipping * g_camRealDistanceFactor); y -= fontHeight * 2;
    }

		if (g_profile)
		{
			DrawImguiString(x, y, Vec3(0.97f, 0.59f, 0.27f), IMGUI_ALIGN_RIGHT, "Total GPU Sim Latency: %.2fms", g_timers.total); y -= fontHeight * 2;
			DrawImguiString(x, y, Vec3(0.0f, 1.0f, 0.0f), IMGUI_ALIGN_RIGHT, "GPU Latencies"); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Predict: %.2fms", g_timers.predict); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Create Cell Indices: %.2fms", g_timers.createCellIndices); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Sort Cell Indices: %.2fms", g_timers.sortCellIndices); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Reorder: %.2fms", g_timers.reorder); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "CreateGrid: %.2fms", g_timers.createGrid); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Collide Particles: %.2fms", g_timers.collideParticles); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Collide Shapes: %.2fms", g_timers.collideShapes); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Collide Triangles: %.2fms", g_timers.collideTriangles); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Calculate Density: %.2fms", g_timers.calculateDensity); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Densities: %.2fms", g_timers.solveDensities); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Velocities: %.2fms", g_timers.solveVelocities); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Rigids: %.2fms", g_timers.solveShapes); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Springs: %.2fms", g_timers.solveSprings); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Inflatables: %.2fms", g_timers.solveInflatables); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Solve Contacts: %.2fms", g_timers.solveContacts); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Apply Deltas: %.2fms", g_timers.applyDeltas); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Finalize: %.2fms", g_timers.finalize); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Triangles: %.2fms", g_timers.updateTriangles); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Normals: %.2fms", g_timers.updateNormals); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Bounds: %.2fms", g_timers.updateBounds); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Calculate Anisotropy: %.2fms", g_timers.calculateAnisotropy); y -= fontHeight;
			DrawImguiString(x, y, Vec3(1.0f), IMGUI_ALIGN_RIGHT, "Update Diffuse: %.2fms", g_timers.updateDiffuse); y -= fontHeight * 2;
		}
    
		x -= 180;

		int uiOffset = 250;
		int uiBorder = 20;
		int uiWidth = 200;
		int uiHeight = g_screenHeight - uiOffset - uiBorder * 3;
		int uiLeft = uiBorder;

		if (g_tweakPanel)
			imguiBeginScrollArea("Scene", uiLeft, g_screenHeight - uiBorder - uiOffset, uiWidth, uiOffset, &g_levelScroll);
		else
			imguiBeginScrollArea("Scene", uiLeft, uiBorder, uiWidth, g_screenHeight - uiBorder - uiBorder, &g_levelScroll);

		if (imguiButton("Reset Scene"))
		{
			g_resetScene = true;
      g_resetSimParams = true;
		}
		imguiSeparatorLine();

		// global options
		imguiLabel("Global");

		if (imguiCheck("Pause", g_pause))
			g_pause = !g_pause;

    if (imguiCheck("Film test", g_filmStepTest)) {
      g_filmStepTest = !g_filmStepTest;
      g_resetScene = true;
    }
    if (imguiCheck("Mesh test", g_meshStepTest)) {
      g_meshStepTest = !g_meshStepTest;
      g_resetScene = true;
    }
    if (imguiCheck("Voxel test", g_voxelStepTest)) {
      g_voxelStepTest = !g_voxelStepTest;
      g_resetScene = true;
    }

		imguiSeparatorLine();
		
		for (int i = 0; i < int(g_scenes.size()); ++i)
		{
			unsigned int color = g_scene == i ? imguiRGBA(255, 151, 61, 255) : imguiRGBA(255, 255, 255, 200);
      if (imguiItem(g_scenes[i]->GetName(), true, color))
      {
				newScene = i;
			}
		}

		imguiEndScrollArea();

		if (g_tweakPanel)
		{
			static int scroll = 0;

			imguiBeginScrollArea("Options", uiLeft, g_screenHeight - uiBorder - uiHeight - uiOffset - uiBorder, uiWidth, uiHeight, &scroll);

      if (imguiCheck("Wireframe", g_wireframe))
      {
				g_wireframe = !g_wireframe;
      }

      if (imguiCheck("Draw Normals", g_drawNormals))
      {
        g_drawNormals = !g_drawNormals;
      }

			if (imguiCheck("Draw Axis", g_drawAxis))
				g_drawAxis = !g_drawAxis;

			if (imguiCheck("Draw Springs", bool(g_drawSprings != 0)))
				g_drawSprings = (g_drawSprings) ? 0 : 1;

      if (imguiCheck("Draw Neighbors", g_drawNeighbors))
        g_drawNeighbors = !g_drawNeighbors;

      if (imguiCheck("Draw Contacts", g_drawContacts))
      {
        g_drawContacts = !g_drawContacts;
      }

			imguiSeparatorLine();

			// scene options
			g_scenes[g_scene]->DoGui();

			imguiSeparatorLine();

      g_numSubstepsAux = float(g_numSubsteps);
      if (imguiSlider("Num Substeps", &g_numSubstepsAux, 0.1f, 10.0f, 1.0f))
      {
        g_numSubsteps = int(g_numSubstepsAux);
        g_resetScene = true;
        g_resetSimParams = false;
      }
      g_numIterationsAux = float(g_params.numIterations);
      if (imguiSlider("Num Iterations", &g_numIterationsAux, 0.1f, 20.0f, 1.0f))
      {
				g_params.numIterations = int(g_numIterationsAux);
        g_resetScene = true;
        g_resetSimParams = false;
      }

			imguiSeparatorLine();
			imguiSlider("Radius", &g_params.radius, 0.01f, 0.5f, 0.01f);
			imguiSlider("Solid Radius", &g_params.solidRestDistance, 0.0f, 0.5f, 0.001f);

			// common params
			imguiSeparatorLine();
			imguiSlider("Dynamic Friction", &g_params.dynamicFriction, 0.0f, 1.0f, 0.01f);
			imguiSlider("Static Friction", &g_params.staticFriction, 0.0f, 1.0f, 0.01f);
			imguiSlider("Particle Friction", &g_params.particleFriction, 0.0f, 1.0f, 0.01f);
			imguiSlider("Restitution", &g_params.restitution, 0.0f, 1.0f, 0.01f);
			imguiSlider("SleepThreshold", &g_params.sleepThreshold, 0.0f, 1.0f, 0.01f);
			imguiSlider("Shock Propagation", &g_params.shockPropagation, 0.0f, 10.0f, 0.01f);
			imguiSlider("Damping", &g_params.damping, 0.0f, 10.0f, 0.01f);
			imguiSlider("Dissipation", &g_params.dissipation, 0.0f, 0.01f, 0.0001f);
			imguiSlider("SOR", &g_params.relaxationFactor, 0.0f, 5.0f, 0.01f);

			imguiSlider("Collision Distance", &g_params.collisionDistance, 0.0f, 0.5f, 0.001f);
			imguiSlider("Collision Margin", &g_params.shapeCollisionMargin, 0.0f, 5.0f, 0.01f);

      imguiSlider("Cam fov", &g_fov, 0.001f, 5, 0.001f);
      imguiSlider("Cam near", &g_camNear, 0.0f, 5.0f, 0.001f);
      imguiSlider("Cam far", &g_camFar, 0.0f, 1500.0f, 0.001f);
      imguiSlider("Cam aspect", &g_aspect, 0.001f, 5.0f, 0.0001f);
      imguiSlider("Cam angle X", &g_camAngle[g_camIndex].x, 0.0f, 2 * M_PI, 0.0001f);
      imguiSlider("Cam angle Y", &g_camAngle[g_camIndex].y, 0.0f, 2 * M_PI, 0.0001f);
      imguiSlider("Cam angle Z", &g_camAngle[g_camIndex].z, 0.0f, 2 * M_PI, 0.0001f);
      imguiSlider("Cam position X", &g_camPos[g_camIndex].x, -10.0f, 10.0f, 0.0001f);
      imguiSlider("Cam position Y", &g_camPos[g_camIndex].y, -10.0f, 10.0f, 0.0001f);
      imguiSlider("Cam position Z", &g_camPos[g_camIndex].z, -10.0f, 10.0f, 0.0001f);

      // cloth params
			imguiSeparatorLine();
			imguiSlider("Drag", &g_params.drag, 0.0f, 1.0f, 0.01f);
			imguiSlider("Lift", &g_params.lift, 0.0f, 1.0f, 0.01f);
			imguiSeparatorLine();
			imguiSlider("Adhesion", &g_params.adhesion, 0.0f, 10.0f, 0.01f);
			imguiSlider("Cohesion", &g_params.cohesion, 0.0f, 0.2f, 0.0001f);
			imguiSlider("Surface Drag", &g_params.freeSurfaceDrag, 0.0f, 1.0f, 0.01f);
			imguiSlider("Buoyancy", &g_params.buoyancy, -1.0f, 1.0f, 0.01f);

			imguiSeparatorLine();

			imguiEndScrollArea();
		}
		imguiEndFrame();

		// kick render commands
		DrawImguiGraph();
	}

	return newScene;
}


void StoreReport(int type, int factor) {
  TestReport result;
  result.type = type;
  result.avgFPS = g_avgFPS;
  result.avgUpdateTime = g_avgUpdateTime * 1000.0f;
  result.avgRenderTime = g_avgRenderTime * 1000.0f;
  result.avgWaitTime = g_avgWaitTime * 1000.0f;  
  result.avgLatencyTime = g_avgLatencyTime * 1000.0f;
  result.factor = factor;
  result.meshVertices = g_meshVertices;
  report.push_back(result);
}

bool isTestingMode() {
  return g_filmStepTest || g_voxelStepTest || g_meshStepTest;
}

void ShowReport() {
  printf("factor\tvertices\tavgUpd\tavgRend\tavgWait\tavgLatency\tavgFPS\n");
  for (int i = 0; i < report.size(); i++) {
    printf("%d\t%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", 
      report[i].factor, 
      report[i].meshVertices, 
      report[i].avgUpdateTime, 
      report[i].avgRenderTime, 
      report[i].avgWaitTime, 
      report[i].avgLatencyTime,
      report[i].avgFPS);
  }
}

void UpdateFrame(bool &quit)
{
	static double lastTime;

	// real elapsed frame time
	double frameBeginTime = GetSeconds();

	g_realdt = float(frameBeginTime - lastTime);
	lastTime = frameBeginTime;

	// do gamepad input polling
	double currentTime = frameBeginTime;
	static double lastJoyTime = currentTime;

	if (g_gamecontroller && currentTime - lastJoyTime > g_dt)
	{
		lastJoyTime = currentTime;

		int leftStickX = SDL_GameControllerGetAxis(g_gamecontroller, SDL_CONTROLLER_AXIS_LEFTX);
		int leftStickY = SDL_GameControllerGetAxis(g_gamecontroller, SDL_CONTROLLER_AXIS_LEFTY);
		int rightStickX = SDL_GameControllerGetAxis(g_gamecontroller, SDL_CONTROLLER_AXIS_RIGHTX);
		int rightStickY = SDL_GameControllerGetAxis(g_gamecontroller, SDL_CONTROLLER_AXIS_RIGHTY);
		int leftTrigger = SDL_GameControllerGetAxis(g_gamecontroller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
		int rightTrigger = SDL_GameControllerGetAxis(g_gamecontroller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

		Vec2 leftStick(joyAxisFilter(leftStickX, 0), joyAxisFilter(leftStickY, 0));
		Vec2 rightStick(joyAxisFilter(rightStickX, 1), joyAxisFilter(rightStickY, 1));
		Vec2 trigger(leftTrigger / 32768.0f, rightTrigger / 32768.0f);

		if (leftStick.x != 0.0f || leftStick.y != 0.0f ||
			rightStick.x != 0.0f || rightStick.y != 0.0f)
		{
			// note constant factor to speed up analog control compared to digital because it is more controllable.
			g_camVel.z = -4 * g_camSpeed * leftStick.y;
			g_camVel.x = 4 * g_camSpeed * leftStick.x;

			// cam orientation
			g_camAngle[g_camIndex].x -= rightStick.x * 0.05f;
			g_camAngle[g_camIndex].y -= rightStick.y * 0.05f;
		}

		// Handle left stick motion
		static bool bLeftStick = false;

		if ((leftStick.x != 0.0f || leftStick.y != 0.0f) && !bLeftStick)
		{
			bLeftStick = true;
		}
		else if ((leftStick.x == 0.0f && leftStick.y == 0.0f) && bLeftStick)
		{
			bLeftStick = false;
			g_camVel.z = -4 * g_camSpeed * leftStick.y;
			g_camVel.x = 4 * g_camSpeed * leftStick.x;
		}

		// Handle triggers as controller button events
		void ControllerButtonEvent(SDL_ControllerButtonEvent event);

		static bool bLeftTrigger = false;
		static bool bRightTrigger = false;
		SDL_ControllerButtonEvent e;

		if (!bLeftTrigger && trigger.x > 0.0f)
		{
			e.type = SDL_CONTROLLERBUTTONDOWN;
			e.button = SDL_CONTROLLER_BUTTON_LEFT_TRIGGER;
			ControllerButtonEvent(e);
			bLeftTrigger = true;
		}
		else if (bLeftTrigger && trigger.x == 0.0f)
		{
			e.type = SDL_CONTROLLERBUTTONUP;
			e.button = SDL_CONTROLLER_BUTTON_LEFT_TRIGGER;
			ControllerButtonEvent(e);
			bLeftTrigger = false;
		}

		if (!bRightTrigger && trigger.y > 0.0f)
		{
			e.type = SDL_CONTROLLERBUTTONDOWN;
			e.button = SDL_CONTROLLER_BUTTON_RIGHT_TRIGGER;
			ControllerButtonEvent(e);
			bRightTrigger = true;
		}
		else if (bRightTrigger && trigger.y == 0.0f)
		{
			e.type = SDL_CONTROLLERBUTTONDOWN;
			e.button = SDL_CONTROLLER_BUTTON_RIGHT_TRIGGER;
			ControllerButtonEvent(e);
			bRightTrigger = false;
		}
	}

	//-------------------------------------------------------------------
	// Scene Update

	double waitBeginTime = GetSeconds();

	MapBuffers(g_buffers);

  double waitEndTime = GetSeconds();

	// Getting timers causes CPU/GPU sync, so we do it after a map
	float newSimLatency = NvFlexGetDeviceLatency(g_solver, &g_GpuTimers.computeBegin, &g_GpuTimers.computeEnd, &g_GpuTimers.computeFreq);
	float newGfxLatency = RendererGetDeviceTimestamps(&g_GpuTimers.renderBegin, &g_GpuTimers.renderEnd, &g_GpuTimers.renderFreq);
	(void)newGfxLatency;

	UpdateCamera();

	if (!(g_complete || g_pause || g_step))
	{
		UpdateScene();
  }

  if (g_generateContactsTexture && g_complete)
  {
    g_generateContactsTexture = false;    // run this block only once
    // this block should execute before unmap command
    // because of read buffer data from comming from gpu to cpu
    BuildReverseTextureMapping();
    BuildColorCompensation(&g_compens_colors[0], &g_buffers->positions[0], g_contact_positions, g_filmDimX, g_filmDimZ);
    //PostProcessReverseTexture();
    //g_drawReverseTexture = true;
  }


	//-------------------------------------------------------------------
	// Render

	double renderBeginTime = GetSeconds();

	if (g_profile && !(g_complete || g_pause || g_step)) 
  {
		if (g_benchmark) 
    {
			g_numDetailTimers = NvFlexGetDetailTimers(g_solver, &g_detailTimers);
		}
		else 
    {
			memset(&g_timers, 0, sizeof(g_timers));
			NvFlexGetTimers(g_solver, &g_timers);
		}
	}

  // main scene render
	StartFrameV2(Vec4(g_clearColor, 1.0f));

  RenderScene();

#ifdef DEBUG_SIM
  RenderDebug(); // this should run beteen map / unmap command, because of reading buffer data incoming from gpu to cpu
#endif

  EndFrameV2();

	int newScene = DoUI();

	// If user has disabled async compute, ensure that no compute can overlap 
	// graphics by placing a sync between them	
	if (!g_useAsyncCompute)
		NvFlexComputeWaitForGraphics(g_flexLib);

	UnmapBuffers(g_buffers);

  // Generate movie output
  if (g_capture)
	{
		TgaImage img;
		img.m_width = g_screenWidth;
		img.m_height = g_screenHeight;
		img.m_data = new uint32_t[g_screenWidth*g_screenHeight];

		ReadFrame((int*)img.m_data, g_screenWidth, g_screenHeight);

		fwrite(img.m_data, sizeof(uint32_t)*g_screenWidth*g_screenHeight, 1, g_ffmpeg);

		delete[] img.m_data;
	}

	double renderEndTime = GetSeconds();

	// if user requested a scene reset process it now
	if (g_resetScene)
	{
		Reset();
		g_resetScene = false;
	}

	//-------------------------------------------------------------------
	// Flex Update

	double updateBeginTime = GetSeconds();

	// send any particle updates to the solver
	NvFlexSetParticles(g_solver, g_buffers->positions.buffer, NULL);
	NvFlexSetVelocities(g_solver, g_buffers->velocities.buffer, NULL);
  NvFlexSetNormals(g_solver, g_buffers->normals.buffer, NULL);
	NvFlexSetPhases(g_solver, g_buffers->phases.buffer, NULL);
	NvFlexSetActive(g_solver, g_buffers->activeIndices.buffer, NULL);
	NvFlexSetActiveCount(g_solver, g_buffers->activeIndices.size());

	// allow scene to update constraints etc
	SyncScene();

	if (g_shapesChanged)
	{
		NvFlexSetShapes(
			g_solver,
			g_buffers->shapeGeometry.buffer,
			g_buffers->shapePositions.buffer,
			g_buffers->shapeRotations.buffer,
			g_buffers->shapePrevPositions.buffer,
			g_buffers->shapePrevRotations.buffer,
			g_buffers->shapeFlags.buffer,
			int(g_buffers->shapeFlags.size()));

		g_shapesChanged = false;
	}

	if (!(g_complete || g_pause || g_step))
	{
		// tick solver
		NvFlexSetParams(g_solver, &g_params);
		NvFlexUpdateSolver(g_solver, g_dt, g_numSubsteps, g_profile);

		g_frame++;
		g_step = false;
	}

	// read back base particle data
	// Note that flexGet calls don't wait for the GPU, they just queue a GPU copy 
	// to be executed later.
	// When we're ready to read the fetched buffers we'll Map them, and that's when
	// the CPU will wait for the GPU flex update and GPU copy to finish.
	NvFlexGetParticles(g_solver, g_buffers->positions.buffer, NULL);
	NvFlexGetVelocities(g_solver, g_buffers->velocities.buffer, NULL);
	NvFlexGetNormals(g_solver, g_buffers->normals.buffer, NULL);
  if (g_buffers->triangles.size()) // dynamic triangles are mentioned when use drag and lift parameters
  {
    NvFlexGetDynamicTriangles(g_solver, g_buffers->triangles.buffer, g_buffers->triangleNormals.buffer, g_buffers->triangles.size() / 3);
  }

	double updateEndTime = GetSeconds();

	//-------------------------------------------------------
	// Update the on-screen timers

	float newUpdateTime = float(updateEndTime - updateBeginTime);
	float newRenderTime = float(renderEndTime - renderBeginTime);
	float newWaitTime = float(waitEndTime - waitBeginTime);
  
	// Exponential filter to make the display easier to read
  const float timerSmoothing = 1.0f;
	g_updateTime = (g_updateTime == 0.0f) ? newUpdateTime : Lerp(g_updateTime, newUpdateTime, timerSmoothing);
	g_renderTime = (g_renderTime == 0.0f) ? newRenderTime : Lerp(g_renderTime, newRenderTime, timerSmoothing);
	g_waitTime = (g_waitTime == 0.0f) ? newWaitTime : Lerp(g_waitTime, newWaitTime, timerSmoothing);
	g_simLatency = (g_simLatency == 0.0f) ? newSimLatency : Lerp(g_simLatency, newSimLatency, timerSmoothing);
  
  if (g_benchmark) newScene = BenchmarkUpdate();

	// flush out the last frame before freeing up resources in the event of a scene change
	// this is necessary for d3d12

  PresentFrame(g_vsync);

	// if gui or benchmark requested a scene change process it now
	if (newScene != -1)
	{
		g_scene = newScene;
		Init(g_scene);
	}

  if (!(g_pause || g_complete)) 
	{	
    // get metrics
    float duration = (g_updateTime + g_renderTime + g_waitTime);
    if (duration  > 0.0f) {
      g_fps = 1.0f/duration;
      frametimes.push_back(g_fps);
      update_times.push_back(g_updateTime);
      render_times.push_back(g_renderTime);
      wait_times.push_back(g_waitTime);      
      latency_times.push_back(g_simLatency);
    }

  }
  else if (g_complete && isTestingMode()) {
    // generate a metrics report
    float totalFPS = 0.0f;
    float totalUpdateTime = 0.0f;
    float totalRenderTime = 0.0f;
    float totalWaitTime = 0.0f;
    float totalLatencyTime = 0.0f;

    int n = frametimes.size();
    for (int i = 0; i < n; i++) {
      totalFPS += frametimes[i];
      totalUpdateTime += update_times[i];
      totalRenderTime += render_times[i];
      totalWaitTime += wait_times[i];
      totalLatencyTime += latency_times[i];
    }
    g_avgFPS = totalFPS / n;
    g_avgRenderTime = totalRenderTime / n;
    g_avgUpdateTime = totalUpdateTime / n;
    g_avgWaitTime = totalWaitTime / n;
    g_avgLatencyTime = totalLatencyTime / n;


    if (g_filmStepTest && g_filmFactor <= g_filmFactorMax) {
      StoreReport(0, g_filmFactor);
      if (g_filmFactor == g_filmFactorMin) {
        g_filmFactor = g_filmFactorStep;
      } else {
        g_filmFactor += g_filmFactorStep;
      }
      g_resetScene = true;
    }
    if (g_meshStepTest && g_meshFactor <= g_meshFactorMax) {
      StoreReport(1, g_meshFactor);
      g_meshFactor += g_meshFactorStep;
      g_resetScene = true;
    }
    if (g_voxelStepTest && g_voxelFactor <= g_meshFactorMax) {
      StoreReport(2, g_voxelFactor);
      g_voxelFactor = int(g_voxelFactor * g_voxelFactorStep); //multiplier factor
      g_resetScene = true;
    }

    if ((g_filmStepTest && g_filmFactor > g_filmFactorMax) || 
        (g_meshStepTest && g_meshFactor > g_meshFactorMax) || 
        (g_voxelStepTest && g_voxelFactor > g_voxelFactorMax)) {
      ShowReport();
      report.resize(0);
      quit = true; // EXIT for call other test in BATCH
    }
  }
}

void PostProcessReverseTexture()
{
  // look for every triangle in the film mesh
  for (int i = 0; i < g_contact_indexes.size() / 3; ++i)
  {   
      int contactIndex = g_contact_indexes[i];
      int countFilledTextCoords = 0;
      int filledIndexes[3] = { -1, -1, -1 };
      Vec4 filledTextureMean = Vec4(0.0f, 0.0f);
      for (int j = 0; j < 3; ++j)
      {
        Vec4 filledTextureSum = Vec4(0.0f, 0.0f);
        if (g_contact_uvs[contactIndex * 3 + j] != -1)
        {
          countFilledTextCoords++;
          filledIndexes[j] = contactIndex * 3 + j;
          filledTextureSum += g_contact_uvs[contactIndex * 3 + j];
        }
        filledTextureMean = filledTextureSum * 0.5f;


      }
      
      if (countFilledTextCoords == 2)
      {
        for (int z = 0; z < 3; z++)
        {
          if (filledIndexes[z] == -1)
          {
            g_contact_uvs[filledIndexes[z]] = filledTextureMean;
          }
        }
      }

      if (countFilledTextCoords == 1)
      {
        filledTextureMean *= 2.0f;
        for (int z = 0; z < 3; z++)
        {
          if (filledIndexes[z] == -1)
          {
            g_contact_uvs[filledIndexes[z]] = filledTextureMean;
          }
        }
      }

  }

}


void BuildReverseTextureMapping()
{
  // get contacts between rigid body and hydrographic film
  const int maxContactsPerParticle = 6;
  NvFlexVector<Vec4> contactPlanes(g_flexLib, g_buffers->positions.size()*maxContactsPerParticle);
  NvFlexVector<Vec4> contactVelocities(g_flexLib, g_buffers->positions.size()*maxContactsPerParticle);
  NvFlexVector<int> contactIndices(g_flexLib, g_buffers->positions.size());
  NvFlexVector<unsigned int> contactCounts(g_flexLib, g_buffers->positions.size());

  NvFlexGetContacts(g_solver, contactPlanes.buffer, contactVelocities.buffer, contactIndices.buffer, contactCounts.buffer);

  // ensure transfers have finished
  contactPlanes.map();
  contactVelocities.map();
  contactIndices.map();
  contactCounts.map();

  // for each active particle of simulation
  for (int i = 0; i < int(g_buffers->activeIndices.size()); ++i)
  {
    // each active particle can have up to 6 contact points on NVIDIA Flex 1.1.0
    const int filmIndex = g_buffers->activeIndices[i];
    Vec3 filmNormal = g_buffers->normals[filmIndex];
    Vec3 filmPosition = Vec3(g_buffers->positions[filmIndex]);
    const int contactIndex = contactIndices[filmIndex];
    const unsigned int count = contactCounts[contactIndex];
    //const float scale = 0.1f;

    //if an active particle have entries in the contact counts array, this particle have contacts
    if (count)
    {
      //retrieve contact planes for each particle 
      // in pratice, the contact plane shows better smoothed reverse texture mapping results
      for (unsigned int c = 0; c < count; ++c)
      {
        Vec4 filmContactPlane = contactPlanes[contactIndex*maxContactsPerParticle + c];
        BuildContactUVs(filmPosition, filmIndex, -Vec3(filmContactPlane), g_gpu_rigid_mesh, g_model, g_contact_positions, g_contact_uvs);
      }
    }
  }

}


#if ENABLE_AFTERMATH_SUPPORT
void DumpAftermathData()
{
	GFSDK_Aftermath_ContextData dataOut;
	GFSDK_Aftermath_Status statusOut;

	NvFlexGetDataAftermath(g_flexLib, &dataOut, &statusOut);
	wprintf(L"Last Aftermath event: %s\n", (wchar_t *)dataOut.markerData);
}
#endif

void ReshapeWindow(int width, int height)
{
	if (!g_benchmark)
		printf("Reshaping\n");

	ReshapeRender(g_window);

	g_screenWidth = width;
	g_screenHeight = height;
  g_aspect = float(g_screenWidth) / g_screenHeight;
}

void ReshapeWindowV2(int width, int height)
{
  if (!g_benchmark)
    printf("Reshaping\n");

  ReshapeRenderV2(g_window);

  g_screenWidth = width;
  g_screenHeight = height;
  g_aspect = float(g_screenWidth) / g_screenHeight;
}

void InputArrowKeysDown(int key, int x, int y)
{
	switch (key)
	{
	case SDLK_DOWN:
	{
		if (g_selectedScene < int(g_scenes.size()) - 1)
			g_selectedScene++;

		// update scroll UI to center on selected scene
		g_levelScroll = max((g_selectedScene - 4) * 24, 0);
		break;
	}
	case SDLK_UP:
	{
		if (g_selectedScene > 0)
			g_selectedScene--;

		// update scroll UI to center on selected scene
		g_levelScroll = max((g_selectedScene - 4) * 24, 0);
		break;
	}
	case SDLK_LEFT:
	{
		if (g_scene > 0)
			--g_scene;
		Init(g_scene);

		// update scroll UI to center on selected scene
		g_levelScroll = max((g_scene - 4) * 24, 0);
		break;
	}
	case SDLK_RIGHT:
	{
		if (g_scene < int(g_scenes.size()) - 1)
			++g_scene;
		Init(g_scene);

		// update scroll UI to center on selected scene
		g_levelScroll = max((g_scene - 4) * 24, 0);
		break;
	}
	}
}

void InputArrowKeysUp(int key, int x, int y)
{
}

bool InputKeyboardDown(unsigned char key, int x, int y)
{
	if (key > '0' && key <= '9')
	{
    if ((key - '0' - 1) >= g_scenes.size())
    {
      cout << "Scene index not exists";
      return false;
    }

		g_scene = key - '0' - 1;
		Init(g_scene);
		return false;
	}

	float kSpeed = g_camSpeed;

	switch (key)
	{
	case 'w':
	{
		g_camVel.z = kSpeed;
		break;
	}
	case 's':
	{
		g_camVel.z = -kSpeed;
		break;
	}
	case 'a':
	{
		g_camVel.x = -kSpeed;
		break;
	}
	case 'd':
	{
		g_camVel.x = kSpeed;
		break;
	}
	case 'q':
	{
		g_camVel.y = kSpeed;
		break;
	}
	case 'z':
	{
		g_camVel.y = -kSpeed;
		break;
	}

	case 'u':
	{
#ifndef ANDROID
		if (g_fullscreen)
		{
			SDL_SetWindowFullscreen(g_window, 0);
			ReshapeWindowV2(1280, 720);
			g_fullscreen = false;
		}
		else
		{
			SDL_SetWindowFullscreen(g_window, SDL_WINDOW_FULLSCREEN_DESKTOP);
			g_fullscreen = true;
		}
#endif
		break;
	}
	case 'r':
	{
		g_resetScene = true;
		break;
	}
	case 'c':
	{
#if _WIN32
		if (!g_ffmpeg)
		{
			// open ffmpeg stream

			int i = 0;
			char buf[255];
			FILE* f = NULL;

			do
			{
				sprintf(buf, "../../movies/output%d.mp4", i);
				f = fopen(buf, "rb");
        if (f) 
        {
					fclose(f);
          cout << "File closed" << endl;
        }
        else 
        {
          cout << "File opened" << endl;
        }

				++i;
			} while (f);

			const char* str = "ffmpeg -r 60 -f rawvideo -pix_fmt rgba -s 1280x720 -i - "
				"-threads 0 -preset fast -y -crf 19 -pix_fmt yuv420p -tune animation -vf vflip %s";

			char cmd[1024];
			sprintf(cmd, str, buf);

			g_ffmpeg = _popen(cmd, "wb");
			assert(g_ffmpeg);
		}
		else
		{
			_pclose(g_ffmpeg);
			g_ffmpeg = NULL;
		}

		g_capture = !g_capture;
		g_frame = 0;
#endif
		break;
	}
  case 'n':
  {
    g_drawNeighbors = !g_drawNeighbors;
    break;
  }
  case 'o':
  {
    g_step = true;
    break;
  }
  case 'p':
	{
		g_pause = !g_pause;
		break;
	}
  case 't':
  {
    g_createReverseTextureFile = !g_createReverseTextureFile;
    break;
  }
  case 'h':
	{
		g_showHelp = !g_showHelp;
		break;
	}
	case 'f':
	{
		g_drawSprings = (g_drawSprings + 1) % 3;
		break;
	}
	case 'i':
	{
		g_drawDiffuse = !g_drawDiffuse;
		break;
	}
	case 'l':
	{
    if (g_camIndex < g_camPos.size() - 1)
    {
      g_camIndex++;
    } 
    else
    {
      g_camIndex = 0; // reset to perspective cam
    }
		break;
	}
	case '.':
	{
		g_profile = !g_profile;
		break;
	}
	case '-':
	{
		if (g_params.numPlanes)
			g_params.numPlanes--;

		break;
	}
	case ';':
	{
		g_debug = !g_debug;
		break;
	}
	case 13:
	{
		g_scene = g_selectedScene;
		Init(g_scene);
		break;
	}
	case 27:
	{
		// return quit = true
		return true;
	}
#if ENABLE_AFTERMATH_SUPPORT
	case 'g':
		DumpAftermathData();
		break;
#endif
	};

	g_scenes[g_scene]->KeyDown(key);

	return false;
}

void InputKeyboardUp(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'w':
	case 's':
	{
		g_camVel.z = 0.0f;
		break;
	}
	case 'a':
	case 'd':
	{
		g_camVel.x = 0.0f;
		break;
	}
	case 'q':
	case 'z':
	{
		g_camVel.y = 0.0f;
		break;
	}
	};
}

void MouseFunc(int b, int state, int x, int y)
{
	switch (state)
	{
	case SDL_RELEASED:
	{
		g_lastx = x;
		g_lasty = y;
		g_lastb = -1;

		break;
	}
	case SDL_PRESSED:
	{
		g_lastx = x;
		g_lasty = y;
		g_lastb = b;
#ifdef ANDROID
		extern void setStateLeft(bool bLeftDown);
		setStateLeft(false);
#endif
		break;
	}
	};
}

void MousePassiveMotionFunc(int x, int y)
{
	g_lastx = x;
	g_lasty = y;
}

void MouseMotionFunc(unsigned state, int x, int y)
{
	float dx = float(x - g_lastx);
	float dy = float(y - g_lasty);

	g_lastx = x;
	g_lasty = y;

	if (state & SDL_BUTTON_RMASK)
	{
		const float kSensitivity = DegToRad(0.1f);
		const float kMaxDelta = FLT_MAX;

		g_camAngle[g_camIndex].x -= Clamp(dx*kSensitivity, -kMaxDelta, kMaxDelta);
		g_camAngle[g_camIndex].y -= Clamp(dy*kSensitivity, -kMaxDelta, kMaxDelta);
	}
}

bool g_Error = false;

void ErrorCallback(NvFlexErrorSeverity severity, const char* msg, const char* file, int line)
{
	printf("Flex: %s - %s:%d\n", msg, file, line);
	g_Error = (severity == eNvFlexLogError);
	//assert(0); asserts are bad for TeamCity
}

void ControllerButtonEvent(SDL_ControllerButtonEvent event)
{
	// map controller buttons to keyboard keys
	if (event.type == SDL_CONTROLLERBUTTONDOWN)
	{
		InputKeyboardDown(GetKeyFromGameControllerButton(SDL_GameControllerButton(event.button)), 0, 0);
		InputArrowKeysDown(GetKeyFromGameControllerButton(SDL_GameControllerButton(event.button)), 0, 0);

		if (event.button == SDL_CONTROLLER_BUTTON_LEFT_TRIGGER)
		{
			// Handle picking events using the game controller
			g_lastx = g_screenWidth / 2;
			g_lasty = g_screenHeight / 2;
			g_lastb = 1;
		}
	}
	else
	{
		InputKeyboardUp(GetKeyFromGameControllerButton(SDL_GameControllerButton(event.button)), 0, 0);
		InputArrowKeysUp(GetKeyFromGameControllerButton(SDL_GameControllerButton(event.button)), 0, 0);

		if (event.button == SDL_CONTROLLER_BUTTON_LEFT_TRIGGER)
		{
			// Handle picking events using the game controller
			g_lastx = g_screenWidth / 2;
			g_lasty = g_screenHeight / 2;
			g_lastb = -1;
		}
	}
}

void ControllerDeviceUpdate()
{
	if (SDL_NumJoysticks() > 0)
	{
		SDL_JoystickEventState(SDL_ENABLE);
		if (SDL_IsGameController(0))
		{
			g_gamecontroller = SDL_GameControllerOpen(0);
		}
	}
}

void SDLInit(const char* title)
{
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0)	// Initialize SDL's Video subsystem and game controllers
		printf("Unable to initialize SDL");

	unsigned int flags = SDL_WINDOW_RESIZABLE;
#if !FLEX_DX
	if (g_graphics == 0)
	{
		SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
		flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL;
	}
#endif

	g_window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		g_screenWidth, g_screenHeight, flags);

	g_windowId = SDL_GetWindowID(g_window);
}

void SDLMainLoop()
{
#if ENABLE_AFTERMATH_SUPPORT
	__try
#endif
	{
	bool quit = false;
	SDL_Event e;
	while (!quit)
	{
		UpdateFrame(quit);

		while (SDL_PollEvent(&e))
		{
			switch (e.type)
			{
			case SDL_QUIT:
				quit = true;
				break;

			case SDL_KEYDOWN:
				if (e.key.keysym.sym < 256 && (e.key.keysym.mod == KMOD_NONE || (e.key.keysym.mod & KMOD_NUM)))
					quit = InputKeyboardDown(e.key.keysym.sym, 0, 0);
				InputArrowKeysDown(e.key.keysym.sym, 0, 0);
				break;

			case SDL_KEYUP:
				if (e.key.keysym.sym < 256 && (e.key.keysym.mod == 0 || (e.key.keysym.mod & KMOD_NUM)))
					InputKeyboardUp(e.key.keysym.sym, 0, 0);
				InputArrowKeysUp(e.key.keysym.sym, 0, 0);
				break;

			case SDL_MOUSEMOTION:
				if (e.motion.state)
					MouseMotionFunc(e.motion.state, e.motion.x, e.motion.y);
				else
					MousePassiveMotionFunc(e.motion.x, e.motion.y);
				break;

			case SDL_MOUSEBUTTONDOWN:
			case SDL_MOUSEBUTTONUP:
				MouseFunc(e.button.button, e.button.state, e.motion.x, e.motion.y);
				break;

			case SDL_WINDOWEVENT:
				if (e.window.windowID == g_windowId)
				{
					if (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
						ReshapeWindowV2(e.window.data1, e.window.data2);
				}
				break;

			case SDL_WINDOWEVENT_LEAVE:
				g_camVel = Vec3(0.0f, 0.0f, 0.0f);
				break;

			case SDL_CONTROLLERBUTTONUP:
			case SDL_CONTROLLERBUTTONDOWN:
				ControllerButtonEvent(e.cbutton);
				break;

			case SDL_JOYDEVICEADDED:
			case SDL_JOYDEVICEREMOVED:
				ControllerDeviceUpdate();
				break;
			}
		}
	}
}
#if ENABLE_AFTERMATH_SUPPORT
	__except (true)
	{
		DumpAftermathData();
	}
#endif
}

int main(int argc, char* argv[])
{
	// process command line args
	for (int i = 1; i < argc; ++i)
	{
		int d;
    if (sscanf(argv[i], "-device=%d", &d))
    {
      g_device = d;
    }

		if (sscanf(argv[i], "-extensions=%d", &d))
			g_extensions = d != 0;

		if (strcmp(argv[i], "-benchmark") == 0)
		{
			g_benchmark = true;
			g_profile = true;
			g_outputAllFrameTimes = false;
			g_vsync = false;
			g_fullscreen = true;
		}

		if (strcmp(argv[i], "-d3d12") == 0)
		{
			g_d3d12 = true;
			// Currently interop doesn't work on d3d12
			g_interop = false;
		}

		if (strcmp(argv[i], "-benchmarkAllFrameTimes") == 0)
		{
			g_benchmark = true;
			g_outputAllFrameTimes = true;
		}

		if (strcmp(argv[i], "-tc") == 0)
		{
			g_teamCity = true;
		}

		if (sscanf(argv[i], "-msaa=%d", &d))
			g_msaaSamples = d;

		int w = 1280;
		int h = 720;
		if (sscanf(argv[i], "-fullscreen=%dx%d", &w, &h) == 2)
		{
			g_screenWidth = w;
			g_screenHeight = h;
			g_fullscreen = true;
		}
		else if (strcmp(argv[i], "-fullscreen") == 0)
		{
			g_screenWidth = w;
			g_screenHeight = h;
			g_fullscreen = true;
		}

		if (sscanf(argv[i], "-windowed=%dx%d", &w, &h) == 2)
		{
			g_screenWidth = w;
			g_screenHeight = h;
			g_fullscreen = false;
		}
		else if (strstr(argv[i], "-windowed"))
		{
			g_screenWidth = w;
			g_screenHeight = h;
			g_fullscreen = false;
		}

    if (sscanf(argv[i], "-vsync=%d", &d) == 1)
      g_vsync = d == 1 ? true : false;
 
		if (sscanf(argv[i], "-multiplier=%d", &d) == 1)
		{
			g_numExtraMultiplier = d;
		}

    float dippingArg = 0.0f;
    if (sscanf(argv[i], "-dippingVelocity=%f", &dippingArg) == 1)
    {
      g_dippingVelocity = dippingArg;
    }

		if (strcmp(argv[i], "-disabletweak") == 0)
		{
			g_tweakPanel = false;
		}

		if (strcmp(argv[i], "-disableinterop") == 0)
		{
			g_interop = false;
		}

		if (sscanf(argv[i], "-asynccompute=%d", &d) == 1)
		{
			g_useAsyncCompute = (d != 0);
		}

		if (sscanf(argv[i], "-graphics=%d", &d) == 1)
		{
			if (d >= 0 && d <= 2)
				g_graphics = d;
		}


    if (strcmp(argv[i], "-testFilm") == 0)
    {
      g_filmStepTest = true;
      g_meshStepTest = false;
      g_voxelStepTest = false;
    }

    if (strcmp(argv[i], "-testMesh") == 0)
    {
      g_filmStepTest = false;
      g_meshStepTest = true;
      g_voxelStepTest = false;
    }

    if (strcmp(argv[i], "-testVoxel") == 0)
    {
      g_filmStepTest = false;
      g_meshStepTest = false;
      g_voxelStepTest = true;
    }

    if (sscanf(argv[i], "-selectedModel=%d", &d) == 0)
    {
      g_selectedModel = d;
    }


  }

	//custom hydrographic scene
	g_scenes.push_back(new Hydrographic("Hydrographic"));

  // init graphics
  RenderInitOptions options;

#ifndef ANDROID
	DemoContext* demoContext = nullptr;
	switch (g_graphics)
	{
	case 0: break;
	case 1: break;
	case 2:
		// workaround for a driver issue with D3D12 with msaa, force it to off
		// options.numMsaaSamples = 1;
		// Currently interop doesn't work on d3d12
		g_interop = false;
		break;
	default: assert(0);
	}

  // Create the demo context
	CreateDemoContext(g_graphics);

	std::string str;
	str = "Hydrographics Simulator v4.2 ";
	switch (g_graphics)
	{
	case 0:
		str += "(Graphics: OpenGL)";
		break;
	case 1:
		str += "(Graphics: DX11)";
		break;
	case 2:
		str += "(Graphics: DX12)";
		break;
	}
	const char* title = str.c_str();

	SDLInit(title);

	options.window = g_window;
	options.numMsaaSamples = g_msaaSamples;
	options.asyncComputeBenchmark = g_asyncComputeBenchmark;
	options.defaultFontHeight = -1;
	options.fullscreen = g_fullscreen;

	InitRender(options);

  if (g_fullscreen)
  {
    SDL_SetWindowFullscreen(g_window, SDL_WINDOW_FULLSCREEN_DESKTOP);
  }

  ReshapeWindowV2(g_screenWidth, g_screenHeight);

#endif // ifndef ANDROID

#if _WIN32 && !FLEX_DX
	// use the PhysX GPU selected from the NVIDIA control panel
	if (g_device == -1)
		g_device = NvFlexDeviceGetSuggestedOrdinal();

	// Create an optimized CUDA context for Flex and set it on the 
	// calling thread. This is an optional call, it is fine to use 
	// a regular CUDA context, although creating one through this API
	// is recommended for best performance.
	bool success = NvFlexDeviceCreateCudaContext(g_device);

	if (!success)
	{
		printf("Error creating CUDA context.\n");
		exit(-1);
	}
#endif

	NvFlexInitDesc desc;
	desc.deviceIndex = g_device;
	desc.enableExtensions = g_extensions;
	desc.renderDevice = 0;
	desc.renderContext = 0;
	desc.computeContext = 0;
	desc.computeType = eNvFlexCUDA;

	// Shared resources are unimplemented on D3D12,
	// so disable it for now.
	if (g_d3d12)
		g_interop = false;

	// Init Flex library, note that no CUDA methods should be called before this 
	// point to ensure we get the device context we want
	g_flexLib = NvFlexInit(NV_FLEX_VERSION, ErrorCallback, &desc);

	if (g_Error || g_flexLib == NULL)
	{
		printf("Could not initialize Flex, exiting.\n");
		exit(-1);
	}

	// store device name
	strcpy(g_deviceName, NvFlexGetDeviceName(g_flexLib));
	printf("Compute Device: %s\n\n", g_deviceName);

	if (g_benchmark)
		g_scene = BenchmarkInit();

	// init default scene
	StartGpuWork();
	Init(g_scene);
	EndGpuWork();

	SDLMainLoop();

	Shutdown();
	DestroyRender();

	SDL_DestroyWindow(g_window);
	SDL_Quit();

	return 0;
}

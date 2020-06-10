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

#pragma once

#include <stdarg.h>
#include <iostream>
#include <iomanip>
#include <algorithm>

// disable some warnings
#if _WIN32
#pragma warning(disable: 4267)  // conversion from 'size_t' to 'int', possible loss of data
#endif

float SampleSDF(const float* sdf, int dim, int x, int y, int z)
{
	assert(x < dim && x >= 0);
	assert(y < dim && y >= 0);
	assert(z < dim && z >= 0);

	return sdf[z*dim*dim + y*dim + x];
}

// return normal of signed distance field
Vec3 SampleSDFGrad(const float* sdf, int dim, int x, int y, int z)
{
	int x0 = max(x-1, 0);
	int x1 = min(x+1, dim-1);

	int y0 = max(y-1, 0);
	int y1 = min(y+1, dim-1);

	int z0 = max(z-1, 0);
	int z1 = min(z+1, dim-1);

	float dx = (SampleSDF(sdf, dim, x1, y, z) - SampleSDF(sdf, dim, x0, y, z))*(dim*0.5f);
	float dy = (SampleSDF(sdf, dim, x, y1, z) - SampleSDF(sdf, dim, x, y0, z))*(dim*0.5f);
	float dz = (SampleSDF(sdf, dim, x, y, z1) - SampleSDF(sdf, dim, x, y, z0))*(dim*0.5f);

	return Vec3(dx, dy, dz);
}

Vec4 getHeatMapColor(float minimum, float maximum, float value)
{
	float ratio = (value - minimum) / (maximum - minimum);
	float b = float(std::max(0.0f, (1 - ratio)));
	float r = float(std::max(0.0f, (ratio - 1)));
	float g = float(1.0f - b - r);
	return Vec4(r, g, b, 1.0f);
}

void GetParticleBounds(Vec3& lower, Vec3& upper)
{
	lower = Vec3(FLT_MAX);
	upper = Vec3(-FLT_MAX);

	for (int i=0; i < g_buffers->positions.size(); ++i)
	{
		lower = Min(Vec3(g_buffers->positions[i]), lower);
		upper = Max(Vec3(g_buffers->positions[i]), upper);
	}
}

void CreateSpring(int i, int j, float stiffness, float give=0.0f)
{
	g_buffers->springIndices.push_back(i);
	g_buffers->springIndices.push_back(j);
	g_buffers->springLengths.push_back((1.0f+give)*Length(Vec3(g_buffers->positions[i])-Vec3(g_buffers->positions[j])));
	g_buffers->springStiffness.push_back(stiffness);	
}

#ifdef TRACK_DISPLACEMENTS
void CreateDisplacementsSpring(int i, int j, float stiffness, float give = 0.0f)
{
  g_displacement_buffers->springIndices.push_back(i);
  g_displacement_buffers->springIndices.push_back(j);
  g_displacement_buffers->springLengths.push_back((1.0f + give)*Length(Vec3(g_displacement_buffers->positions[i]) - Vec3(g_displacement_buffers->positions[j])));
  g_displacement_buffers->springStiffness.push_back(stiffness);
}
#endif;

void AddSphere(float radius, Vec3 position, Quat rotation)
{
	NvFlexCollisionGeometry geo;
	geo.sphere.radius = radius;
	g_buffers->shapeGeometry.push_back(geo);

	g_buffers->shapePositions.push_back(Vec4(position, 0.0f));
	g_buffers->shapeRotations.push_back(rotation);

	g_buffers->shapePrevPositions.push_back(g_buffers->shapePositions.back());
	g_buffers->shapePrevRotations.push_back(g_buffers->shapeRotations.back());

	int flags = NvFlexMakeShapeFlags(eNvFlexShapeSphere, false);
	g_buffers->shapeFlags.push_back(flags);
}

void CreateSDF(const Mesh* mesh, uint32_t dim, Vec3 lower, Vec3 upper, float* sdf)
{
	if (mesh)
	{
		printf("Begin mesh voxelization\n");

		double startVoxelize = GetSeconds();

		uint32_t* volume = new uint32_t[dim*dim*dim];
		Voxelize((const Vec3*)&mesh->m_positions[0], mesh->m_positions.size(), (const int*)&mesh->m_indices[0], mesh->m_indices.size(), dim, dim, dim, volume, lower, upper);

		printf("End mesh voxelization (%.2fs)\n", (GetSeconds()-startVoxelize));
	
		printf("Begin SDF gen (fast marching method)\n");

		double startSDF = GetSeconds();

		MakeSDF(volume, dim, dim, dim, sdf);

    //MakeSDFCUDA(volume, dim, dim, dim, sdf); // not so performatic

		printf("End SDF gen (%.2fs)\n", (GetSeconds()-startSDF));
	
		delete[] volume;
	}
}

NvFlexTriangleMeshId CreateTriangleMesh(Mesh* m)
{
	if (!m)
		return 0;

	Vec3 lower, upper;
	m->GetBounds(lower, upper);

	NvFlexVector<Vec4> positions(g_flexLib, m->m_positions.size());
	positions.map();
	NvFlexVector<int> indices(g_flexLib);

	for (int i = 0; i < int(m->m_positions.size()); ++i)
	{
		Vec3 vertex = Vec3(m->m_positions[i]);
		positions[i] = Vec4(vertex, 0.0f);
	}
	indices.assign((int*)&m->m_indices[0], m->m_indices.size());

	positions.unmap();
	indices.unmap();

	NvFlexTriangleMeshId flexMesh = NvFlexCreateTriangleMesh(g_flexLib);
	NvFlexUpdateTriangleMesh(g_flexLib, flexMesh, positions.buffer, indices.buffer, m->GetNumVertices(), m->GetNumFaces(), (float*)&lower, (float*)&upper);

	// entry in the collision->render map
	g_meshes[flexMesh] = CreateGpuMesh(m);
	
	return flexMesh;
}

void AddTriangleMesh(NvFlexTriangleMeshId mesh, Vec3 translation, Quat rotation, Vec3 scale)
{
	Vec3 lower, upper;
	NvFlexGetTriangleMeshBounds(g_flexLib, mesh, lower, upper);

	NvFlexCollisionGeometry geo;
	geo.triMesh.mesh = mesh;
	geo.triMesh.scale[0] = scale.x;
	geo.triMesh.scale[1] = scale.y;
	geo.triMesh.scale[2] = scale.z;

	g_buffers->shapePositions.push_back(Vec4(translation, 0.0f));
	g_buffers->shapeRotations.push_back(Quat(rotation));
	g_buffers->shapePrevPositions.push_back(Vec4(translation, 0.0f));
	g_buffers->shapePrevRotations.push_back(Quat(rotation));
	g_buffers->shapeGeometry.push_back((NvFlexCollisionGeometry&)geo);
	g_buffers->shapeFlags.push_back(NvFlexMakeShapeFlags(eNvFlexShapeTriangleMesh, false));
}
/*
NvFlexDistanceFieldId CreateSDF(const char* meshFile, int dim, float margin = 0.1f, float expand = 0.0f)
{
	Mesh* mesh = ImportMesh(meshFile);

	// include small margin to ensure valid gradients near the boundary
	mesh->Normalize(1.0f - margin);
	mesh->Transform(TranslationMatrix(Point3(margin, margin, margin)*0.5f));

	Vec3 lower(0.0f);
	Vec3 upper(1.0f);

	string sdfFile = string(meshFile, strrchr(meshFile, '.')) + ".pfm";

	PfmImage pfm;
	if (!PfmLoad(sdfFile.c_str(), pfm))
	{
		pfm.m_width = dim;
		pfm.m_height = dim;
		pfm.m_depth = dim;
		pfm.m_data = new float[dim*dim*dim];

		printf("Cooking SDF: %s - dim: %d^3\n", sdfFile.c_str(), dim);

		CreateSDF(mesh, dim, lower, upper, pfm.m_data);

		PfmSave(sdfFile.c_str(), pfm);
	}

	printf("Loaded SDF, %d\n", pfm.m_width);

	assert(pfm.m_width == pfm.m_height && pfm.m_width == pfm.m_depth);

	// cheap collision offset
	int numVoxels = int(pfm.m_width*pfm.m_height*pfm.m_depth);
	for (int i = 0; i < numVoxels; ++i)
		pfm.m_data[i] += expand;

	NvFlexVector<float> field(g_flexLib);
	field.assign(pfm.m_data, pfm.m_width*pfm.m_height*pfm.m_depth);
	field.unmap();

	// set up flex collision shape
	NvFlexDistanceFieldId sdf = NvFlexCreateDistanceField(g_flexLib);
	NvFlexUpdateDistanceField(g_flexLib, sdf, dim, dim, dim, field.buffer);

	// entry in the collision->render map
	g_fields[sdf] = CreateGpuMesh(mesh);

	delete mesh;
	delete[] pfm.m_data;

	return sdf;
}
*/
NvFlexDistanceFieldId CreateHydrographicSDF(Mesh* mesh, const char* meshFile, bool createSDFFile, int dim, Mat44 transformation, float margin = 0.1f, float expand = 0.0f)
{
	if (!mesh)
		throw std::runtime_error(std::string("Import mesh fail!"));

	//apply a transformation before normalize and generate the sdf
	//because of mesh->Normalize call, only the rotation transform can be preserved
	//with sdf mesh, other transformatiosn can be done scaling and translating the other models of the scene
	mesh->Transform(transformation);
	// include small margin to ensure valid gradients near the boundary
	mesh->Normalize(1.0f - margin);
	mesh->Transform(TranslationMatrix(Point3(margin, margin, margin)*0.5f));

	Vec3 lower(0.0f);
	Vec3 upper(1.0f);

	// Begin Add Android Support
#ifdef ANDROID
	string sdfFile = string(meshFile, strlen(meshFile) - strlen(strrchr(meshFile, '.'))) + ".pfm";
#else
	string sdfFile = string(meshFile, strrchr(meshFile, '.')) + ".pfm";
#endif
	// End Add Android Support
	
	PfmImage pfm;
	bool pfmLoaded = PfmLoad(sdfFile.c_str(), pfm); // try and load the sdf in pfm format from disc if it exists
	if (createSDFFile || !pfmLoaded || pfm.m_width != dim)
	{
 		pfm.m_width = dim;
		pfm.m_height = dim;
		pfm.m_depth = dim;
		pfm.m_data = new float[dim*dim*dim];

		printf("Cooking SDF: %s - dim: %d^3\n", sdfFile.c_str(), dim);

		CreateSDF(mesh, dim, lower, upper, pfm.m_data);
		// ...then save in pfm format, to use in the next program perform
		PfmSave(sdfFile.c_str(), pfm);
	}

	printf("Loaded SDF, %d\n", pfm.m_width);

	assert(pfm.m_width == pfm.m_height && pfm.m_width == pfm.m_depth);

	// cheap collision offset
	int numVoxels = int(pfm.m_width*pfm.m_height*pfm.m_depth);
	for (int i = 0; i < numVoxels; ++i)
		pfm.m_data[i] += expand;

	NvFlexVector<float> field(g_flexLib);
	field.assign(pfm.m_data, pfm.m_width*pfm.m_height*pfm.m_depth);
	field.unmap();

	// set up flex collision shape
	NvFlexDistanceFieldId sdf = NvFlexCreateDistanceField(g_flexLib);
	NvFlexUpdateDistanceField(g_flexLib, sdf, dim, dim, dim, field.buffer);


	delete mesh;
	delete[] pfm.m_data;

	return sdf;
}

void AddSDF(NvFlexDistanceFieldId sdf, Vec3 translation, Quat rotation, float width)
{
	NvFlexCollisionGeometry geo;
	geo.sdf.field = sdf;
	geo.sdf.scale = width;

	g_buffers->shapePositions.push_back(Vec4(translation, 0.0f));
	g_buffers->shapeRotations.push_back(Quat(rotation));
	g_buffers->shapePrevPositions.push_back(Vec4(translation, 0.0f));
	g_buffers->shapePrevRotations.push_back(Quat(rotation));
	g_buffers->shapeGeometry.push_back((NvFlexCollisionGeometry&)geo);
	g_buffers->shapeFlags.push_back(NvFlexMakeShapeFlags(eNvFlexShapeSDF, false));
}

inline int GridIndex(int x, int y, int dx) { return y*dx + x; }

void CreateSpringGrid(Vec3 lower, int dx, int dy, int dz, float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity, float invMass)
{
	int baseIndex = int(g_buffers->positions.size());

	for (int z=0; z < dz; ++z)
	{
		for (int y=0; y < dy; ++y)
		{
			for (int x=0; x < dx; ++x)
			{
				Vec3 position = lower + radius*Vec3(float(x), float(z), float(y));

				g_buffers->positions.push_back(Vec4(position.x, position.y, position.z, invMass));
				g_buffers->velocities.push_back(velocity);
				g_buffers->phases.push_back(phase);

				if (x > 0 && y > 0)
				{
					g_buffers->triangles.push_back(baseIndex + GridIndex(x-1, y-1, dx));
					g_buffers->triangles.push_back(baseIndex + GridIndex(x, y-1, dx));
					g_buffers->triangles.push_back(baseIndex + GridIndex(x, y, dx));
					
					g_buffers->triangles.push_back(baseIndex + GridIndex(x-1, y-1, dx));
					g_buffers->triangles.push_back(baseIndex + GridIndex(x, y, dx));
					g_buffers->triangles.push_back(baseIndex + GridIndex(x-1, y, dx));

					g_buffers->triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
					g_buffers->triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
				}
			}
		}
	}	

	// horizontal
	for (int y=0; y < dy; ++y)
	{
		for (int x=0; x < dx; ++x)
		{
			int index0 = y*dx + x;

			if (x > 0)
			{
				int index1 = y*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (x > 1)
			{
				int index2 = y*dx + x - 2;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}

			if (y > 0 && x < dx-1)
			{
				int indexDiag = (y-1)*dx + x + 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}

			if (y > 0 && x > 0)
			{
				int indexDiag = (y-1)*dx + x - 1;
				CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
			}
		}
	}

	// vertical
	for (int x=0; x < dx; ++x)
	{
		for (int y=0; y < dy; ++y)
		{
			int index0 = y*dx + x;

			if (y > 0)
			{
				int index1 = (y-1)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
			}

			if (y > 1)
			{
				int index2 = (y-2)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}
		}
	}	
}

//hyperbolic clip
// https://stackoverflow.com/questions/9323903/most-efficient-elegant-way-to-clip-a-number
float clip(float x, float min, float max) {
	return ((max - min) / 2)*((exp(x) - exp(-x)) / (exp(x) + exp(-x))) + max - (max - min) / 2;
}

float computeStiffness(float r, float rMax, float x, float x0, float z, float z0) {
	int mode = 5;
	float a, b, c, f;
	float factor = 0.02f;

	switch (mode) {
		case 0:
			//f(r) = a(r) + b
			a = -5.0f / rMax;
			b = 1.0f;
			f = a * r + b;
			break;
		case 1:
			//f(r) = a(r)^2 + b
			a = -20.0f;
			b = 1.0f;
			f = a * sqr(r) + b;
			break;
		case 2:
			//f(x, z) = (x-x0)^2/a + (z-z0)^2/b + c
			a = -7.0f * factor;
			b = -11.0f * factor;
			c = 2.0f;
			f = sqr(x - x0)/a + sqr(z- z0)/b + c;
			break;
		case 3:
			//f(x, z) = (x-x0)^2/a + (z-z0)^2/b + c
			a = -0.25f;
			b = -0.25f;
			c = 1.0f;
			f = sqr(x - x0) / a + sqr(z - z0) / b + c;
			break;
		default:
			f = 0.4f;
			break;
	}

	f = clip(f, 0.01f, 1.0f);

	return f;
}

float computeDistance(Vec3 center, Vec3 position) {
	return Length(center - position);
}

void CreateHydrographicSpringGrid(Vec3 lower, Vec3 meshCenter, int dx, int dy, int dz, 
	float radius, int phase, float stretchStiffness, float bendStiffness, float shearStiffness, 
	Vec3 velocity, float invMass, float invMassV = 1.0f, float invMassH = 1.0f, 
	bool hasBendShiftess = false, bool hasShearShtifness = false)
{
	int baseIndex = int(g_buffers->positions.size());
	Vec3 higher = lower + radius * Vec3(float(dx), float(dz), float(dy));
	Vec3 distortionCenter = meshCenter;

	for (int z = 0; z < dz; ++z)
	{
		for (int y = 0; y < dy; ++y)
		{
			for (int x = 0; x < dx; ++x)
			{
				Vec3 position = lower + radius*Vec3(float(x), float(z), float(y));
				if (x == 0 || x == dx - 1) {
					g_buffers->positions.push_back(Vec4(position.x, position.y, position.z, invMassV));// 0.08f aumenta o arrasto nas bordas
        }
				else if (y == 0 || y== dy - 1) {
					g_buffers->positions.push_back(Vec4(position.x, position.y, position.z, invMassH));// 0.08f aumenta o arrasto nas bordas
        }
				else {
					g_buffers->positions.push_back(Vec4(position.x, position.y, position.z, invMass));
        }
        g_buffers->normals.push_back(Vec4(0.0f, 1.0f, 0.0f, 0.0f));
				g_buffers->velocities.push_back(velocity);
				g_buffers->phases.push_back(phase);

				// texture coordinates
				float u = (float)x / (dx - 1);
				float v = (float)y / (dy - 1);
				g_buffers->uvs.push_back(Vec4(u, v, 0.0f, 0.0f));

        // triangle indexes
				if (x > 0 && y > 0)
				{
          /*
          Film mesh topology
          *   *   *
          | \ | / |
          *   *   *
          | / | \ |
          *   *   *
          */
          if ((x % 2 == 1 && y % 2 == 1) || (x % 2 == 0 && y % 2 == 0))
          {
            /*
            Four plane vertices produces two inverted triangles
              v1 v3
              | \ |
              v2 v4
            */
            // 1st triangle
            g_buffers->triangles.push_back(baseIndex + GridIndex(x, y, dx));
            g_buffers->triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
            g_buffers->triangles.push_back(baseIndex + GridIndex(x, y - 1, dx));
            // 2nd triangle
            g_buffers->triangles.push_back(baseIndex + GridIndex(x - 1, y, dx));
            g_buffers->triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
            g_buffers->triangles.push_back(baseIndex + GridIndex(x, y, dx));
          }
          else 
          {
            /*
            Four plane vertices produces two inverted triangles
            v1 v3
            | / |
            v2 v4
            */
            // 1st triangle
            g_buffers->triangles.push_back(baseIndex + GridIndex(x - 1, y, dx));
            g_buffers->triangles.push_back(baseIndex + GridIndex(x - 1, y - 1, dx));
            g_buffers->triangles.push_back(baseIndex + GridIndex(x, y - 1, dx));
            // 2nd triangle
            g_buffers->triangles.push_back(baseIndex + GridIndex(x - 1, y, dx));
            g_buffers->triangles.push_back(baseIndex + GridIndex(x, y - 1, dx));
            g_buffers->triangles.push_back(baseIndex + GridIndex(x, y, dx));
          }

          g_buffers->triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));
          g_buffers->triangleNormals.push_back(Vec3(0.0f, 1.0f, 0.0f));

        }
			}
		}
	}

	// horizontal
	for (int y = 0; y < dy; ++y)
	{
		for (int x = 0; x < dx; ++x)
		{
			int index0 = y*dx + x;

			if (x > 0)
			{
				int index1 = y*dx + x - 1;
				//Vec4 mean = 0.5f * (g_buffers->positions[baseIndex + index0] + g_buffers->positions[baseIndex + index1]);
				//float d = computeDistance(distortionCenter, Vec3(mean));
				//CreateSpring(baseIndex + index0, baseIndex + index1, computeStiffness(d, min(dx * radius, dy * radius), mean.x, distortionCenter.x, mean.z, distortionCenter.z));
        CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
      }

			if (hasBendShiftess && x > 1)
			{
				int index2 = y*dx + x - 2;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}

      
      if (hasShearShtifness && y > 0 && x < dx - 1)
      {
        int indexDiag = (y - 1)*dx + x + 1;
        CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
      }

      if (hasShearShtifness && y > 0 && x > 0)
      {
        int indexDiag = (y - 1)*dx + x - 1;
        CreateSpring(baseIndex + index0, baseIndex + indexDiag, shearStiffness);
      }
			
		}
	}
	// vertical
	for (int x = 0; x < dx; ++x)
	{
		for (int y = 0; y < dy; ++y)
		{
			int index0 = y*dx + x;

			if (y > 0)
			{
				int index1 = (y - 1)*dx + x;
				//Vec4 mean = 0.5f * (g_buffers->positions[baseIndex + index0] + g_buffers->positions[baseIndex + index1]);
				//float d = computeDistance(distortionCenter, Vec3(mean));
				//CreateSpring(baseIndex + index0, baseIndex + index1, computeStiffness(d, min(dx * radius, dy * radius), mean.x, distortionCenter.x, mean.z, distortionCenter.z));
        CreateSpring(baseIndex + index0, baseIndex + index1, stretchStiffness);
      }

			if (hasBendShiftess && y > 1)
			{
				int index2 = (y - 2)*dx + x;
				CreateSpring(baseIndex + index0, baseIndex + index2, bendStiffness);
			}
		}
	}

}


namespace
{
	struct Tri
	{
		int a;
		int b;
		int c;

		Tri(int a, int b, int c) : a(a), b(b), c(c) {}

		bool operator < (const Tri& rhs)
		{
			if (a != rhs.a)
				return a < rhs.a;
			else if (b != rhs.b)
				return b < rhs.b;
			else
				return c < rhs.c;
		}
	};
}


namespace
{
	struct TriKey
	{
		int orig[3];
		int indices[3];

		TriKey(int a, int b, int c)		
		{
			orig[0] = a;
			orig[1] = b;
			orig[2] = c;

			indices[0] = a;
			indices[1] = b;
			indices[2] = c;

			std::sort(indices, indices+3);
		}			

		bool operator < (const TriKey& rhs) const
		{
			if (indices[0] != rhs.indices[0])
				return indices[0] < rhs.indices[0];
			else if (indices[1] != rhs.indices[1])
				return indices[1] < rhs.indices[1];
			else
				return indices[2] < rhs.indices[2];
		}
	};
}

void CreateTetMesh(const char* filename, Vec3 lower, float scale, float stiffness, int phase)
{
	FILE* f = fopen(filename, "r");

	char line[2048];

	if (f)
	{
		typedef std::map<TriKey, int> TriMap;
		TriMap triCount;

		const int vertOffset = g_buffers->positions.size();

		Vec3 meshLower(FLT_MAX);
		Vec3 meshUpper(-FLT_MAX);

		bool firstTet = true;

		while (!feof(f))
		{
			if (fgets(line, 2048, f))
			{
				switch(line[0])
				{
				case '#':
					break;
				case 'v':
					{
						Vec3 pos;
						sscanf(line, "v %f %f %f", &pos.x, &pos.y, &pos.z);

						g_buffers->positions.push_back(Vec4(pos.x, pos.y, pos.z, 1.0f));
						g_buffers->velocities.push_back(0.0f);
						g_buffers->phases.push_back(phase);

						meshLower = Min(pos, meshLower);
						meshUpper = Max(pos, meshUpper);
						break;
					}
				case 't':
					{
						if (firstTet)
						{
							Vec3 edges = meshUpper-meshLower;
							float maxEdge = max(edges.x, max(edges.y, edges.z));

							// normalize positions
							for (int i=vertOffset; i < int(g_buffers->positions.size()); ++i)
							{
								Vec3 p = lower + (Vec3(g_buffers->positions[i])-meshLower)*scale/maxEdge;
								g_buffers->positions[i] = Vec4(p, g_buffers->positions[i].w);
							}

							firstTet = false;
						}

						int indices[4];
						sscanf(line, "t %d %d %d %d", &indices[0], &indices[1], &indices[2], &indices[3]);

						indices[0] += vertOffset;
						indices[1] += vertOffset;
						indices[2] += vertOffset;
						indices[3] += vertOffset;

						CreateSpring(indices[0], indices[1], stiffness);
						CreateSpring(indices[0], indices[2], stiffness);
						CreateSpring(indices[0], indices[3], stiffness);
				
						CreateSpring(indices[1], indices[2], stiffness);
						CreateSpring(indices[1], indices[3], stiffness);
						CreateSpring(indices[2], indices[3], stiffness);

						TriKey k1(indices[0], indices[2], indices[1]);
						triCount[k1] += 1;

						TriKey k2(indices[1], indices[2], indices[3]);
						triCount[k2] += 1;

						TriKey k3(indices[0], indices[1], indices[3]);
						triCount[k3] += 1;

						TriKey k4(indices[0], indices[3], indices[2]);
						triCount[k4] += 1;

						break;
					}
				}
			}
		}

		for (TriMap::iterator iter=triCount.begin(); iter != triCount.end(); ++iter)
		{
			TriKey key = iter->first;

			// only output faces that are referenced by one tet (open faces)
			if (iter->second == 1)
			{
				g_buffers->triangles.push_back(key.orig[0]);
				g_buffers->triangles.push_back(key.orig[1]);
				g_buffers->triangles.push_back(key.orig[2]);
				g_buffers->triangleNormals.push_back(0.0f);
			}
		}


		fclose(f);
	}
}


void DrawImguiString(int x, int y, Vec3 color, int align, const char* s, ...)
{
	char buf[2048];

	va_list args;

	va_start(args, s);
	vsnprintf(buf, 2048, s, args);
	va_end(args);

	imguiDrawText(x, y, align, buf, imguiRGBA((unsigned char)(color.x*255), (unsigned char)(color.y*255), (unsigned char)(color.z*255)));
}

// Soft body support functions

Vec3 CalculateMean(const Vec3* particles, const int* indices, int numIndices)
{
	Vec3 sum;

	for (int i = 0; i < numIndices; ++i)
		sum += Vec3(particles[indices[i]]);

	if (numIndices)
		return sum / float(numIndices);
	else
		return sum;
}

float CalculateRadius(const Vec3* particles, Vec3 center, const int* indices, int numIndices)
{
	float radiusSq = 0.0f;

	for (int i = 0; i < numIndices; ++i)
	{
		float dSq = LengthSq(Vec3(particles[indices[i]]) - center);
		if (dSq > radiusSq)
			radiusSq = dSq;
	}

	return sqrtf(radiusSq);
}

struct Cluster
{
	Vec3 mean;
	float radius;

	// indices of particles belonging to this cluster
	std::vector<int> indices;
};

struct Seed
{
	int index;
	float priority;

	bool operator < (const Seed& rhs) const
	{
		return priority < rhs.priority;
	}
};

int CreateClusters(Vec3* particles, const float* priority, int numParticles, std::vector<int>& outClusterOffsets, std::vector<int>& outClusterIndices, std::vector<Vec3>& outClusterPositions, float radius, float smoothing = 0.0f)
{
	std::vector<Seed> seeds;
	std::vector<Cluster> clusters;

	// flags a particle as belonging to at least one cluster
	std::vector<bool> used(numParticles, false);

	// initialize seeds
	for (int i = 0; i < numParticles; ++i)
	{
		Seed s;
		s.index = i;
		s.priority = priority[i];

		seeds.push_back(s);
	}

	std::stable_sort(seeds.begin(), seeds.end());

	while (seeds.size())
	{
		// pick highest unused particle from the seeds list
		Seed seed = seeds.back();
		seeds.pop_back();

		if (!used[seed.index])
		{
			Cluster c;

			const float radiusSq = sqr(radius);

			// push all neighbors within radius
			for (int p = 0; p < numParticles; ++p)
			{
				float dSq = LengthSq(Vec3(particles[seed.index]) - Vec3(particles[p]));
				if (dSq <= radiusSq)
				{
					c.indices.push_back(p);

					used[p] = true;
				}
			}

			c.mean = CalculateMean(particles, &c.indices[0], c.indices.size());

			clusters.push_back(c);
		}
	}

	if (smoothing > 0.0f)
	{
		// expand clusters by smoothing radius
		float radiusSmoothSq = sqr(smoothing);

		for (int i = 0; i < int(clusters.size()); ++i)
		{
			Cluster& c = clusters[i];

			// clear cluster indices
			c.indices.resize(0);

			// push all neighbors within radius
			for (int p = 0; p < numParticles; ++p)
			{
				float dSq = LengthSq(c.mean - Vec3(particles[p]));
				if (dSq <= radiusSmoothSq)
					c.indices.push_back(p);
			}

			c.mean = CalculateMean(particles, &c.indices[0], c.indices.size());
		}
	}

	// write out cluster indices
	int count = 0;

	//outClusterOffsets.push_back(0);

	for (int c = 0; c < int(clusters.size()); ++c)
	{
		const Cluster& cluster = clusters[c];

		const int clusterSize = int(cluster.indices.size());

		// skip empty clusters
		if (clusterSize)
		{
			// write cluster indices
			for (int i = 0; i < clusterSize; ++i)
				outClusterIndices.push_back(cluster.indices[i]);

			// write cluster offset
			outClusterOffsets.push_back(outClusterIndices.size());

			// write center
			outClusterPositions.push_back(cluster.mean);

			++count;
		}
	}

	return count;
}

// creates distance constraints between particles within some distance
int CreateLinks(const Vec3* particles, int numParticles, std::vector<int>& outSpringIndices, std::vector<float>& outSpringLengths, std::vector<float>& outSpringStiffness, float radius, float stiffness = 1.0f)
{
	const float radiusSq = sqr(radius);

	int count = 0;

	for (int i = 0; i < numParticles; ++i)
	{
		for (int j = i + 1; j < numParticles; ++j)
		{
			float dSq = LengthSq(Vec3(particles[i]) - Vec3(particles[j]));

			if (dSq < radiusSq)
			{
				outSpringIndices.push_back(i);
				outSpringIndices.push_back(j);
				outSpringLengths.push_back(sqrtf(dSq));
				outSpringStiffness.push_back(stiffness);

				++count;
			}
		}
	}

	return count;
}

void CreateSkinning(const Vec3* vertices, int numVertices, const Vec3* clusters, int numClusters, float* outWeights, int* outIndices, float falloff, float maxdist)
{
	const int maxBones = 4;

	// for each vertex, find the closest n clusters
	for (int i = 0; i < numVertices; ++i)
	{
		int indices[4] = { -1, -1, -1, -1 };
		float distances[4] = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
		float weights[maxBones];

		for (int c = 0; c < numClusters; ++c)
		{
			float dSq = LengthSq(vertices[i] - clusters[c]);

			// insertion sort
			int w = 0;
			for (; w < maxBones; ++w)
				if (dSq < distances[w])
					break;

			if (w < maxBones)
			{
				// shuffle down
				for (int s = maxBones - 1; s > w; --s)
				{
					indices[s] = indices[s - 1];
					distances[s] = distances[s - 1];
				}

				distances[w] = dSq;
				indices[w] = c;
			}
		}

		// weight particles according to distance
		float wSum = 0.0f;

		for (int w = 0; w < maxBones; ++w)
		{
			if (distances[w] > sqr(maxdist))
			{
				// clamp bones over a given distance to zero
				weights[w] = 0.0f;
			}
			else
			{
				// weight falls off inversely with distance
				weights[w] = 1.0f / (powf(distances[w], falloff) + 0.0001f);
			}

			wSum += weights[w];
		}

		if (wSum == 0.0f)
		{
			// if all weights are zero then just 
			// rigidly skin to the closest bone
			weights[0] = 1.0f;
		}
		else
		{
			// normalize weights
			for (int w = 0; w < maxBones; ++w)
			{
				weights[w] = weights[w] / wSum;
			}
		}

		// output
		for (int j = 0; j < maxBones; ++j)
		{
			outWeights[i*maxBones + j] = weights[j];
			outIndices[i*maxBones + j] = indices[j];
		}
	}
}


void SampleMesh(Mesh* mesh, Vec3 lower, Vec3 scale, float rotation, float radius, float volumeSampling, float surfaceSampling, std::vector<Vec3>& outPositions)
{
	if (!mesh)
		return;

	mesh->Transform(RotationMatrix(rotation, Vec3(0.0f, 1.0f, 0.0f)));

	Vec3 meshLower, meshUpper;
	mesh->GetBounds(meshLower, meshUpper);

	Vec3 edges = meshUpper - meshLower;
	float maxEdge = max(max(edges.x, edges.y), edges.z);

	// put mesh at the origin and scale to specified size
	Matrix44 xform = ScaleMatrix(scale / maxEdge)*TranslationMatrix(Point3(-meshLower));

	mesh->Transform(xform);
	mesh->GetBounds(meshLower, meshUpper);

	std::vector<Vec3> samples;

	if (volumeSampling > 0.0f)
	{
		// recompute expanded edges
		edges = meshUpper - meshLower;
		maxEdge = max(max(edges.x, edges.y), edges.z);

		// use a higher resolution voxelization as a basis for the particle decomposition
		float spacing = radius / volumeSampling;

		// tweak spacing to avoid edge cases for particles laying on the boundary
		// just covers the case where an edge is a whole multiple of the spacing.
		float spacingEps = spacing*(1.0f - 1e-4f);

		// make sure to have at least one particle in each dimension
		int dx, dy, dz;
		dx = spacing > edges.x ? 1 : int(edges.x / spacingEps);
		dy = spacing > edges.y ? 1 : int(edges.y / spacingEps);
		dz = spacing > edges.z ? 1 : int(edges.z / spacingEps);

		int maxDim = max(max(dx, dy), dz);

		// expand border by two voxels to ensure adequate sampling at edges
		meshLower -= 2.0f*Vec3(spacing);
		meshUpper += 2.0f*Vec3(spacing);
		maxDim += 4;

		vector<uint32_t> voxels(maxDim*maxDim*maxDim);

		// we shift the voxelization bounds so that the voxel centers
		// lie symmetrically to the center of the object. this reduces the 
		// chance of missing features, and also better aligns the particles
		// with the mesh
		Vec3 meshOffset;
		meshOffset.x = 0.5f * (spacing - (edges.x - (dx - 1)*spacing));
		meshOffset.y = 0.5f * (spacing - (edges.y - (dy - 1)*spacing));
		meshOffset.z = 0.5f * (spacing - (edges.z - (dz - 1)*spacing));
		meshLower -= meshOffset;

		//Voxelize(*mesh, dx, dy, dz, &voxels[0], meshLower - Vec3(spacing*0.05f) , meshLower + Vec3(maxDim*spacing) + Vec3(spacing*0.05f));
		Voxelize((const Vec3*)&mesh->m_positions[0], mesh->m_positions.size(), (const int*)&mesh->m_indices[0], mesh->m_indices.size(), maxDim, maxDim, maxDim, &voxels[0], meshLower, meshLower + Vec3(maxDim*spacing));

		// sample interior
		for (int x = 0; x < maxDim; ++x)
		{
			for (int y = 0; y < maxDim; ++y)
			{
				for (int z = 0; z < maxDim; ++z)
				{
					const int index = z*maxDim*maxDim + y*maxDim + x;

					// if voxel is marked as occupied the add a particle
					if (voxels[index])
					{
						Vec3 position = lower + meshLower + spacing*Vec3(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f);

						// normalize the sdf value and transform to world scale
						samples.push_back(position);
					}
				}
			}
		}
	}

	// move back
	mesh->Transform(ScaleMatrix(1.0f)*TranslationMatrix(Point3(-0.5f*(meshUpper + meshLower))));
	mesh->Transform(TranslationMatrix(Point3(lower + 0.5f*(meshUpper + meshLower))));

	if (surfaceSampling > 0.0f)
	{
		// sample vertices
		for (int i = 0; i < int(mesh->m_positions.size()); ++i)
			samples.push_back(Vec3(mesh->m_positions[i]));

		// random surface sampling
		if (1)
		{
			for (int i = 0; i < 50000; ++i)
			{
				int t = Rand() % mesh->GetNumFaces();
				float u = Randf();
				float v = Randf()*(1.0f - u);
				float w = 1.0f - u - v;

				int a = mesh->m_indices[t * 3 + 0];
				int b = mesh->m_indices[t * 3 + 1];
				int c = mesh->m_indices[t * 3 + 2];
				
				Point3 pt = mesh->m_positions[a] * u + mesh->m_positions[b] * v + mesh->m_positions[c] * w;
				Vec3 p(pt.x,pt.y,pt.z);

				samples.push_back(p);
			}
		}
	}

	std::vector<int> clusterIndices;
	std::vector<int> clusterOffsets;
	std::vector<Vec3> clusterPositions;
	std::vector<float> priority(samples.size());

	CreateClusters(&samples[0], &priority[0], samples.size(), clusterOffsets, clusterIndices, outPositions, radius);

}

void ClearShapes()
{
	g_buffers->shapeGeometry.resize(0);
	g_buffers->shapePositions.resize(0);
	g_buffers->shapeRotations.resize(0);
	g_buffers->shapePrevPositions.resize(0);
	g_buffers->shapePrevRotations.resize(0);
	g_buffers->shapeFlags.resize(0);
}

void UpdateShapes()
{	
	// mark shapes as dirty so they are sent to flex during the next update
	g_shapesChanged = true;
}

// calculates the union bounds of all the collision shapes in the scene
void GetShapeBounds(Vec3& totalLower, Vec3& totalUpper)
{
	Bounds totalBounds;

	for (int i=0; i < g_buffers->shapeFlags.size(); ++i)
	{
		NvFlexCollisionGeometry geo = g_buffers->shapeGeometry[i];

		int type = g_buffers->shapeFlags[i]&eNvFlexShapeFlagTypeMask;

		Vec3 localLower;
		Vec3 localUpper;

		switch(type)
		{
			case eNvFlexShapeBox:
			{
				localLower = -Vec3(geo.box.halfExtents);
				localUpper = Vec3(geo.box.halfExtents);
				break;
			}
			case eNvFlexShapeSphere:
			{
				localLower = -geo.sphere.radius;
				localUpper = geo.sphere.radius;
				break;
			}
			case eNvFlexShapeCapsule:
			{
				localLower = -Vec3(geo.capsule.halfHeight, 0.0f, 0.0f) - Vec3(geo.capsule.radius);
				localUpper = Vec3(geo.capsule.halfHeight, 0.0f, 0.0f) + Vec3(geo.capsule.radius);
				break;
			}
			case eNvFlexShapeConvexMesh:
			{
				NvFlexGetConvexMeshBounds(g_flexLib, geo.convexMesh.mesh, localLower, localUpper);

				// apply instance scaling
				localLower *= geo.convexMesh.scale;
				localUpper *= geo.convexMesh.scale;
				break;
			}
			case eNvFlexShapeTriangleMesh:
			{
				NvFlexGetTriangleMeshBounds(g_flexLib, geo.triMesh.mesh, localLower, localUpper);
				
				// apply instance scaling
				localLower *= Vec3(geo.triMesh.scale);
				localUpper *= Vec3(geo.triMesh.scale);
				break;
			}
			case eNvFlexShapeSDF:
			{
				localLower = 0.0f;
				localUpper = geo.sdf.scale;
				break;
			}
		};

		// transform local bounds to world space
		Vec3 worldLower, worldUpper;
		TransformBounds(localLower, localUpper, Vec3(g_buffers->shapePositions[i]), g_buffers->shapeRotations[i], 1.0f, worldLower, worldUpper);

		totalBounds = Union(totalBounds, Bounds(worldLower, worldUpper));
	}

	totalLower = totalBounds.lower;
	totalUpper = totalBounds.upper;
}
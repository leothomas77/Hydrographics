//#define DEBUG_GRID
#define PATH_SIZE 256

class ModelData {
public:
	ModelData(char *path, Vec3 translation, float scale, Vec3 rotation):
  translation{translation}, scale{scale}, rotation{rotation}
  {
		if (strlen(path) > (PATH_SIZE - 1))
      throw std::runtime_error(std::string("Model name too long!"));
    strcpy(this->path, path);
	}
  char path[PATH_SIZE];
	Vec3 translation = Vec3(0.0f);
	float scale = 0.0f;
	Vec3 rotation;
};

class Hydrographic : public Scene
{
public:

	Hydrographic(const char* name) :
	  Scene(name) {}
	/**
		M�todo para o ajuste das constantes para calibrar o simulador
		O modelo SDF ter� sempre um bounding box de 1x1, podendo ser aplicada rota��o. Com a quina para a origem dos eixos.
		0.0 Executar o experimento f�sico com alinhamento (usando nivel de bolha) e com aparelho eletromec�nico, com conten��es de bordas para o filme.
			0.0 Nivelar qualquer rota��o em x e z. Em y poss�velmente poder� ocorrer, mas pode ser desfeita virtualmente.
		0. Medir com escala em cm a rela��o entre o tam dos quadrados na folha plana e o modelo real.
		1. Fotografar o modelo real com o xadrez alinhado .
		2. Rodar a simula��o, sem a colis�o, com o modelo atravessando o filme. -> no metodo Update, comentar UpdateShapes();
			2.1 Alinhar o modelo para que a lateral coincida com a borda do filme (transla��o do filme)
			2.2	Ajustar o fator de escala da lateral do quadrado para coincidir com a medi��o real.
			Caso n�o coincida, rodar novamente at� coincidir.
		3. Posicionar o filme novamente para que o modelo fique ao centro
			3.1 Rodar a simula��o, e alinhar x e z do filme para coincidir no ponto de colis�o do xadrez 
			3.2 Verificar a necessidade de rota��o em (x, y, z), de acordo com pontos de refer�ncia da maior dimens�o x, y, z da imagem e seu alinhamento com a linha x e z do xadrez
			3.2 Caso n�o ajuste, modificar: stretch, fator de relaxamento, drag e lift de forma a modificar o esticamento
			3.3 Ainda � poss�vel modificar a fixa��o das bordas variando de 0 a 1.
		4. Trazer a imagem para um editor de imagens em camadas. 
			4.1 Trazer a foto nivelada para o editor de imagens em camadas
			4.2 Trazer a foto da simula��o para o editor de imagens em camadas, com transpar�ncia de 30 a 50%
			4.3 Comparar os pontos do xadrez da foto com a imagem e tirar as medidas

	*/
	virtual void Initialize()
	{	//-0.012f

    //modelos centro
		  ModelData tetra = ModelData("../../data/tetrahedron2.obj", Vec3(0.0f, 0.0f, 0.0f), 0.216f, Vec3(0.0f, 90.0f, 0.0f));//fator escala medido 0.216
      ModelData sphere = ModelData("../../data/sphere.obj", Vec3(0.0f, 0.0f, 0.0f), 0.3f, Vec3(0.0f)); //.17
      ModelData turtle = ModelData("../../data/tartaruga_centro.obj", Vec3(0.0f, 0.0f, 0.0f), 0.125f, Vec3(0.0f, -95.5f, 0.0f)); //fator escala fixa medida 0.14
      ModelData onca = ModelData("../../data/Onca_Poisson_15_16_0.obj", Vec3(0.0f, -1.8f, 0.0f), 0.125f, Vec3(0.0f, 0.0f, 0.0f)); //fator escala fixa medida 0.14
      ModelData bunny = ModelData("../../data/bunny.obj", Vec3(0.0f, 0.0f, 0.0f), 0.125f, Vec3(0.0f, -95.5f, 0.0f)); //fator escala fixa medida 0.14
      ModelData earth = ModelData("../../data/Earth.obj", Vec3(0.0f, 0.0f, 0.0f), 0.5f, Vec3(0.0f, 0.0f, 0.0f)); //fator escala fixa medida 0.14

    //modelos alinhados - N�O REMOVER                                                                                                                                 //modelos alinhados trasladados
      //ModelData tetra = ModelData("../../data/tetrahedron2.obj", Vec3(-0.02f, 0.0f, 0.3f), 0.216f, Vec3(0.0f, -90.0f, 0.0f));//fator escala medido 0.216
		  //ModelData sphere = ModelData("../../data/sphere.obj", Vec3(0.0f, 0.0f, 0.0f), 0.17f, Vec3(0.0f)); //.17
		  //ModelData turtle = ModelData("../../data/tartaruga_centro.obj", Vec3(0.09f, 0.0f, 0.1f), 0.125f, Vec3(0.0f, -95.5f, 0.0f)); //fator escala fixa medida 0.14
		  //ModelData turtleRot = ModelData("tartaruga_centro.obj", Vec3(-0.07f, 0.0f, 0.2f), 0.130f, Vec3(0.0f, -95.53f, 0.0f));

		models.clear();
		models.push_back(tetra);
		models.push_back(sphere);
		models.push_back(turtle);
    models.push_back(onca);
    models.push_back(bunny);
    models.push_back(earth);

    if (g_selectedModel >= 0 && g_selectedModel < models.size())
    {
      selectedModel = g_selectedModel;
    }
    else
    {
      // without texture
      //0: tetraedro 1: sphere 2: turtle
      // with texture
      //3: onca 4: bunny 5: earth 
      selectedModel = 5;
    }


    g_fps = 0.0f;
    g_frame = 0;
		//float stretchStiffness = 1.0f; // default tartaruga esfera tetra
    float stretchStiffness = 1.0f; // onca: 0.2 
    float bendStiffness = 0.75f; // not used
		float shearStiffness = 0.5f; // not used
		verticalInvMass = 1.0f;
		horizontalInvMass = 1.0f;

		//float radius = 0.05f;

		//int dimx = factor * A4_Width;
		//int dimz = factor * A4_Height;
		//float spacing = spacingFactor * A4_Spacing;//radius*0.8f;
		// para world map: dimx = 10 dimz = 5
#ifdef DEBUG_GRID
		// para debug alinhamento posicoes
		factor = 1;
		int dimx = factor * (4+1);
		int dimz = factor * (4+1);
		float spacing = .25f;
#else
    A4_Spacing = models[selectedModel].scale;
    if (g_filmStepTest) {
      factor = g_filmFactor;
    }
    if (g_meshStepTest) {
      sphereSectors = g_meshFactor;
    }
    if (g_voxelStepTest) {
      voxelDim = g_voxelFactor;
    }

		gridDimX = factor * (A4_Width + 1);
		gridDimZ = factor * (A4_Height + 1);
		float spacing = A4_Spacing / factor;
#endif // DEBUG_GRID
		gridSpacing = spacing;

		gridPosition = Vec3(-(gridDimX - 1)*spacing*0.5f, gridY, -(gridDimZ - 1)*spacing*0.5f);

		int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
		
		// general params
    g_params.numIterations = 5; //onca: 3
    g_numSubsteps = 2;
    // g_params.numIterations = 5; // iteracoes pbd default

    g_params.gravity[0] = 0.0f; // gravidade x, y, z
		g_params.gravity[1] = 0.0f;
		g_params.gravity[2] = 0.0f;
		// common params
		g_params.dynamicFriction = 1.0f;// coefs de friccao - altos valores substituem o recurso de ades�o
		g_params.staticFriction = 1.0;
		g_params.particleFriction = 1.0f;
		g_params.restitution = 0.0f; // coef. restituicao (colisao totalmente inelastica, corpos seguem juntos)
		//g_params.adhesion = 0.032f; // coef de ades�o
		//g_params.sleepThreshold

		//g_params.maxSpeed
		//g_params.maxAcceleration

		//g_params.shockPropagation = 0.3f;
		//g_params.dissipation = 0.25f;
		//g_params.damping = 0.5f;

		// fluid params
		g_params.viscosity = 0.0f;
		// cloth params
    g_params.drag = .2f;//0.3f;//3.5f;//força de arrasto ao tecido. aumenta sensa��o de elasticidade
    g_params.lift = .8f;//0.2f;//1.8f;// forca de sustentação
		// collision params
    float radius = 0.012f;
    g_params.radius = radius;//0.012f;   //radius*1.0f; // raio max de intera��o entre part�culas
    g_params.collisionDistance = 0.012f;	//dist�ncia mantida entre a part�cula e o shape ap�s colis�o
		g_params.particleCollisionMargin = radius*0.5f;//
		g_params.shapeCollisionMargin = radius*0.5f;//
		g_params.numPlanes = 0;
		// relaxation solver params
		//g_params.relaxationMode = eNvFlexRelaxationGlobal;
		//g_params.relaxationFactor = 0.25f;
		g_params.relaxationMode = eNvFlexRelaxationLocal;
		g_params.relaxationFactor = 0.2f;
		// plastic params
		//g_params.plasticThreshold = 0.1f;
		//g_params.plasticCreep = 0.3f;

		//g_windStrength = 0.0f;

		//tetrahedron2.obj (rotacionado x) tetrahedron1.obj
		//sphere.obj sphere960f
		//tartaruga_centro.obj
		//happy_vrip.ply ATENCAO!!! .ply
		Mat44 mTrans = TranslationMatrix(translation);
		angle = models[selectedModel].rotation;
		Mat44 mRotX = RotationMatrix(DegToRad(angle.x), Vec3(1.0f, 0.0f, 0.0f));
		Mat44 mRotY = RotationMatrix(DegToRad(angle.y), Vec3(0.0f, 1.0f, 0.0f));
		Mat44 mRotZ = RotationMatrix(DegToRad(angle.z), Vec3(0.0f, 0.0f, 1.0f));
		Mat44 mRot = mRotZ * mRotY * mRotX;
		Mat44 mScale = ScaleMatrix(Vec3(scale, scale, scale)); //tartaruga z:0.96f
		Mat44 transf = mTrans * mRot * mScale;

		Vec3 pos = Vec3(0.0f, initialY, 0.0f);
		Quat rot = QuatFromAxisAngle(Vec3(1.0f), 0.0f);
    // define model matrix to start simulation
    g_model = TranslationMatrix(Point3(pos))*RotationMatrix(Quat(rot))*ScaleMatrix(1.0f);

    Mesh* mesh = NULL; // mesh model for Flex simulator
    GpuMesh* gpuMesh = NULL; // mesh for GPU and rendering
    if (g_meshStepTest) {
      cout << "Selecting sphere mesh with " << sphereSectors << " segments and " << sphereSectors << " slices " << endl;
      mesh = CreateSphere(sphereSectors, sphereSectors, 1.0f);
    } else {
      mesh = ImportMesh(GetFilePathByPlatform(models[selectedModel].path).c_str());
    }

    triangles = mesh->m_triangles; // copy the triangle array
    triangleIndexes = mesh->m_triangle_index;
    cout << "Mesh positions:" << mesh->m_positions.size() << endl;
    cout << "Mesh triangles:" << mesh->m_indices.size()/3 << endl;

    // define collision mesh type
    if (1) {
		  sdfMesh = CreateHydrographicSDF(mesh, GetFilePathByPlatform(models[selectedModel].path).c_str(), createFile, voxelDim, transf, sdfMargin, expand);
			AddSDF(sdfMesh, pos, rot, 1.0f);

		}
		else {
      // para usar esta malha,tem que ajustar as transformacoes de escala
		  triangleMesh = CreateTriangleMesh(ImportMesh(GetFilePathByPlatform(models[selectedModel].path).c_str()));
		  AddTriangleMesh(triangleMesh, pos, rot, 1.0f);
		}

    // entry in the collision->render map
#ifdef RENDER_V2
    // gpu mesh created by assimp import optimized for loading a full texture atlas features and rendering
    g_gpu_mesh = CreateGpuMesh(GetFilePathByPlatform(models[selectedModel].path).c_str(), transf, sdfMargin);
#else
    g_fields[sdf] = CreateGpuMesh(mesh);
#endif

		meshIndex = g_buffers->shapePositions.size() - 1;
		//get the sdf center after the normalization
		GetShapeBounds(lower, upper);
		g_meshCenter = (lower + upper) * 0.5f;

		gridPosition.x += g_meshCenter.x + models[selectedModel].translation.x;
		gridPosition.z += g_meshCenter.z + models[selectedModel].translation.z;


    Matrix44 filmModel = TranslationMatrix(Point3(gridPosition));

		CreateHydrographicSpringGrid(gridPosition, g_meshCenter, gridDimX, gridDimZ, 1, spacing, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f, horizontalInvMass, verticalInvMass, false, false);
    //filmContactCount.resize(g_buffers->positions.size());
    for (int i = 0; i < g_buffers->positions.size(); i++)
    {
      //filmContactCount[i] = 0;
      g_buffers->uvs[i] = Vec4(-1.0f);
    }
    
#ifdef RENDER_V2
    g_film_mesh = CreateGpuFilm(filmModel, &g_buffers->positions[0], &g_buffers->normals[0], g_buffers->uvs.size() ? &g_buffers->uvs[0] : NULL, g_buffers->positions.size(), &g_buffers->triangles[0], g_buffers->triangles.size());
#endif    
    // draw options
		g_drawSprings = false;
		g_drawHydrographic = true;
		g_drawShadows = false;
    g_generateContactsTexture = true;
		mTime = 0.0f;
	}

 	void Update()
	{
		mTime += g_dt;

		float time = Max(0.0f, mTime-startTime);
		float lastTime = Max(0.0f, time-g_dt);

		GetShapeBounds(lower, upper);

		// stop animation when hidrographics is complete
		if (upper.y + 0.2f < gridY)
		{
			g_pause = true;
		}

#ifdef TRACK_DISPLACEMENTS // old way for tracking distortion
    float epsilon = 0.0f;
    float displacementThreshold = 0.0f;
    float displacementFactor = 1.0f;
    if (lower.y <= gridY && !g_pause)  // is dipping
    {
		  if (0) // CUDA way
		  {
			  Vec4* d_positions;
			  Vec4* d_disp_positions;
			  Vec4* d_originalPositions;
			  Vec3* d_velocities;

			  size_t nPositions = g_buffers->positions.size();

			  cudaMalloc(&d_positions, nPositions * sizeof(Vec4));
			  cudaMalloc(&d_disp_positions, nPositions * sizeof(Vec4));
			  cudaMalloc(&d_velocities, nPositions * sizeof(Vec3));
			  cudaMalloc(&d_originalPositions, nPositions * sizeof(Vec4));

			  cudaMemcpy(d_positions, &g_buffers->positions[0], nPositions * sizeof(Vec4), cudaMemcpyHostToDevice);
			  cudaMemcpy(d_disp_positions, &g_displacement_buffers->positions[0], nPositions * sizeof(Vec4), cudaMemcpyHostToDevice);
			  cudaMemcpy(d_velocities, &g_displacement_buffers->velocities[0], nPositions * sizeof(Vec3), cudaMemcpyHostToDevice);
			  cudaMemcpy(d_originalPositions, &g_displacement_buffers->originalPositions[0], nPositions * sizeof(Vec4), cudaMemcpyHostToDevice);

			  UpdateDisplacements(nPositions, 1, d_positions, d_disp_positions, d_velocities, d_originalPositions);

			  //cudaDeviceSynchronize();

			  cudaMemcpy(&g_displacement_buffers->positions[0], d_disp_positions, nPositions * sizeof(Vec4), cudaMemcpyDeviceToHost);
			  cudaMemcpy(&g_displacement_buffers->velocities[0], d_velocities, nPositions * sizeof(Vec3), cudaMemcpyDeviceToHost);

			  cudaFree(d_positions);
			  cudaFree(d_disp_positions);
			  cudaFree(d_velocities);
			  cudaFree(d_originalPositions);
		  }

		  if (0) // CPU way
		  {
        NvFlexVector<int> currentActiveIndices(g_flexLib);
        currentActiveIndices.resize(0);
        currentActiveIndices.reserve(g_buffers->activeIndices.size());
        int activeCount = 0;
		    for (int activeIndex = 0; activeIndex < g_buffers->activeIndices.size(); activeIndex++)
		    {
          int i = g_buffers->activeIndices[activeIndex];

			    Vec3 originalPos = g_displacement_buffers->originalPositions[i];
			    Vec3 displacedPos = Vec3(g_buffers->positions[i]);
			    float dipping = fabs(g_buffers->positions[i].y - gridY);
			    float displacement = Length(originalPos - displacedPos);
			    if (dipping <= epsilon && displacement > displacementThreshold) // particle is not yet dipped
			    {
			      //track position
            currentActiveIndices.push_back(i);
            activeCount++;
			      Vec3 gradient = (originalPos - displacedPos) / displacement;
			      Vec3 predictedPosition = displacementFactor * displacement * gradient;
			      g_displacement_buffers->positions[i].x += predictedPosition.x;
			      g_displacement_buffers->positions[i].y += predictedPosition.y;
			      g_displacement_buffers->positions[i].z += predictedPosition.z;
			      //g_displacement_buffers->velocities[i] = (originalPos - displacedPos) / g_dt;
			    }
			    else {
			      g_displacement_buffers->velocities[i] = Vec3(0.0f, 0.0f, 0.0f);
			    }
		    }

        /*
        // Remove collided vertices from active indices to improve peformance
        NvFlexBuffer* activeBuffer = NvFlexAllocBuffer(g_flexLib, currentActiveIndices.count, sizeof(int), eNvFlexBufferHost);
        int* activeIndices = (int*)NvFlexMap(activeBuffer, eNvFlexMapWait);

        for (int i = 0; i < currentActiveIndices.count; ++i)
          activeIndices[i] = currentActiveIndices[i];

        NvFlexUnmap(activeBuffer);

        NvFlexCopyDesc copyDesc;
        copyDesc.dstOffset = 0;
        copyDesc.srcOffset = 0;
        copyDesc.elementCount = currentActiveIndices.count;

        NvFlexSetActive(g_solver, activeBuffer, &copyDesc);
        */
		  }
    }
#endif

		Vec3 pos = Vec3(0.0f, initialY + dippingVelocity * time, 0.0f);
		Vec3 prevPos = Vec3(0.0f, initialY + dippingVelocity * lastTime, 0.0f);

		Quat rot = QuatFromAxisAngle(Vec3(1.0f), 0.0f);
		Quat prevRot = QuatFromAxisAngle(Vec3(1.0f), 0.0f);
    
    // updates model matrix for rendering
    g_model = TranslationMatrix(Point3(pos))*RotationMatrix(Quat(rot))*ScaleMatrix(1.0f);

		g_buffers->shapePositions[meshIndex] = Vec4(pos, 0.0f);
		g_buffers->shapeRotations[meshIndex] = rot;

		g_buffers->shapePrevPositions[meshIndex] = Vec4(prevPos, 0.0f);
		g_buffers->shapePrevRotations[meshIndex] = prevRot;

		UpdateShapes(); // update data to flex check colision

   
		if (0)
		{
			//get contacted particles
			int  maxContactsPerParticle = 6;
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

			// each active particle of simulation
			for (int i = 0; i < int(g_buffers->activeIndices.size()); ++i)
			{
				// each active particle can have up to 6 contact points on NVIDIA Flex 1.1.0
				const int contactIndex = contactIndices[g_buffers->activeIndices[i]];
				const unsigned int count = contactCounts[contactIndex];

				Vec3 position = g_buffers->positions[g_buffers->activeIndices[i]];
        float positionW = g_buffers->positions[g_buffers->activeIndices[i]].w;
				if (!count && position.y < gridY) // not touched particle
				{
					Vec3 normal = g_buffers->normals[g_buffers->activeIndices[i]];
          //normal.y = 0;
          Normalize(normal);

          Vec3 newPosition = position - normal * (g_dt / g_numSubsteps);// parei aqui

         // newPosition.y = gridY;
         // g_buffers->velocities[g_buffers->activeIndices[i]] = Length(Vec3(newPosition - position)) * normal / (time - lastTime);

					g_buffers->positions[g_buffers->activeIndices[i]] = Vec4(newPosition, positionW);

				}				
        else 
        {
					// each active particle can have up to 6 contact points on NVIDIA Flex 1.1.0
					//retrieve contact planes for each particle 
					for (unsigned int c = 0; c < count; ++c)
					{
						g_buffers->velocities[g_buffers->activeIndices[i]] = Vec3(0.0f, dippingVelocity, 0.0f);
						Vec4 position = g_buffers->positions[g_buffers->activeIndices[i]];

						position.y += dippingVelocity * g_dt;
						position.w = 0.0f; //total adhesion

						g_buffers->positions[g_buffers->activeIndices[i]] = position;
					}
				}
        
			}
		}

  }

	void DoGui()
	{
		imguiLabel("Hydrographic options");
		if (imguiCheck("Create SDF file", createFile))
			createFile = !createFile;
		imguiSlider("Dipping velocity", &dippingVelocity, -0.5f, 0.5f, 0.005f);
		imguiSlider("Horizontal invMass", &horizontalInvMass, 0.0f, 1.0f, 0.005f);
		imguiSlider("Vertical invMass", &verticalInvMass, 0.0f, 1.0f, 0.005f);

		if (imguiCheck("Draw Hydrographic Film", g_drawHydrographic))
			g_drawHydrographic = !g_drawHydrographic;

		if (imguiCheck("Draw Collision Mesh", g_drawHydrographicCollisionMesh))
			g_drawHydrographicCollisionMesh = !g_drawHydrographicCollisionMesh;

		//imguiSlider("Grid Translation X", &models[selectedModel].translation[0], -5.0f, 5.0f, 0.001f);
		//imguiSlider("Grid Translation Z", &models[selectedModel].translation[2], -5.0f, 5.0f, 0.001f);
		//imguiSlider("Rotation angle X ", &angle.x, -90.0f, 90.0f, 10.0f);
		//imguiSlider("Rotation angle Y ", &angle.y, -90.0f, 90.0f, 10.0f);
		//imguiSlider("Rotation angle Z ", &angle.z, -90.0f, 90.0f, 10.0f);

    float floatVoxelDim = float(voxelDim);
    if (imguiSlider("Voxel dim", &floatVoxelDim, 63.0f, 512.0f, 8.0f)) {
      voxelDim = int(floatVoxelDim);
    }

    float floatSectors = float(sphereSectors);
    if (imguiSlider("Model sphere sectors", &floatSectors, 0.0f, 450.0f, 10.0f)) {
      sphereSectors = int(floatSectors);
    }

    float floatFactor = float(factor);
    if (imguiSlider("Grid scale factor", &floatFactor, 0.0f, 45.0f, 1.0f)) {
      factor = int(floatFactor);
    }
		imguiSeparatorLine();

		for (int i = 0; i < int(models.size()); ++i)
		{
			unsigned int color = g_scene == i ? imguiRGBA(255, 151, 61, 255) : imguiRGBA(255, 255, 255, 200);
      if(imguiItem(models[i].path, true, color))
			{
				selectedModel = i;
			}
		}
	}

	//Time params
  float mTime;
  float startTime = 1.0f; //GetSeconds();
	//SDF mesh params
	bool createFile = true;
	int voxelDim = 128;
	float expand = 0.0f;
	float sdfMargin = 0.05f;
  //model params
	Vec3 lower, upper;
	int meshIndex = 0;
	int selectedModel = 0;
  float initialY = 1.5f;
	NvFlexDistanceFieldId sdfMesh = NULL;
	NvFlexTriangleMeshId triangleMesh = NULL;
  int sphereSectors = 160;
	//Film params
  float gridY = 0.3f; // grid position
  int factor = 4;//16; 
  float A4_Spacing = 0.17f;//default spacing based on sphere model
  int A4_Width = 7;//7; //210
  int A4_Height = 7;//11; //297
	//Vec2 boundsX, boundsY, boundsZ;
	Point3 translation = Point3(0.0f);
	float scale = 1.0f;
	Vec3 angle = Vec3(0.0f);
	std::vector<ModelData> models;
  bool tracking = false;
  //
  std::vector<Triangle> triangles;
  std::vector<TriangleIndexes> triangleIndexes;
};
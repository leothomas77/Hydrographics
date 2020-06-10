//#define DEBUG_GRID

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
      // para usar esta malha, tem que ajustar as transformacoes de escala
		  triangleMesh = CreateTriangleMesh(ImportMesh(GetFilePathByPlatform(models[selectedModel].path).c_str()));
		  AddTriangleMesh(triangleMesh, pos, rot, 1.0f);

		}

    // gpu mesh created by assimp import optimized for loading a full texture atlas features and rendering
    g_gpu_rigid_mesh = CreateGpuMesh(GetFilePathByPlatform(models[selectedModel].path).c_str(), transf, sdfMargin);

    // get texture pixels for use in the fix seam stage
    //g_rigid_model_texture_pixels = new u32
    
    //= new unsigned byte[g_rigid_texture_width * g_rigid_texture_height * 4];


		meshIndex = g_buffers->shapePositions.size() - 1;
		//get the sdf center after the normalization
		GetShapeBounds(lower, upper);
		g_meshCenter = (lower + upper) * 0.5f;

		gridPosition.x += g_meshCenter.x + models[selectedModel].translation.x;
		gridPosition.z += g_meshCenter.z + models[selectedModel].translation.z;

    Matrix44 filmModel = TranslationMatrix(Point3(gridPosition));

		CreateHydrographicSpringGrid(gridPosition, g_meshCenter, gridDimX, gridDimZ, 1, spacing, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f, horizontalInvMass, verticalInvMass, false, false);

    // initialize contact structures for generate reverse texture mapping
    g_contact_positions.resize(g_buffers->positions.size());
    g_contact_normals.resize(g_buffers->normals.size());
    g_contact_indexes.resize(g_buffers->triangles.size());
    g_contact_uvs.resize(g_buffers->positions.size());
    // create a copy of flat film positinos before deformation occurs
    // do the same to normals and triangles array
    g_buffers->positions.copyto(&g_contact_positions[0], g_buffers->positions.size());
    for (int i = 0; i < g_buffers->positions.size(); i++)
    {
      g_contact_uvs[i] = Vec4(-1.0f); // initialize contact uv with -1 means this vertice has not contact with rigid body
    }
    g_buffers->normals.copyto(&g_contact_normals[0], g_buffers->normals.size());
    g_buffers->triangles.copyto(&g_contact_indexes[0], g_buffers->triangles.size());
    
    // flex data is only handle the interaction between the soft bodies
    // handle the positions and collisions
    // data structure for render the scene and show texture mapping properly is provided by gpu_mesh structure created here
    // first time this structure is created and the buffers allocated here
    // this same structure is updated in the render procedure in the main loop, with the positions updated by flex
    g_gpu_film_mesh = CreateGpuFilm(filmModel, &g_buffers->positions[0], &g_buffers->normals[0], g_buffers->uvs.size() ? &g_buffers->uvs[0] : NULL, g_buffers->positions.size(), &g_buffers->triangles[0], g_buffers->triangles.size());

    // draw options
		g_drawSprings = false;
		g_drawHydrographic = true;
		g_drawShadows = false;
    g_generateContactsTexture = true;
    g_drawReverseTexture = true;
    g_drawFixedSeams = true;
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
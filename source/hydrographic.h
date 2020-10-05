
#define SQUARES_W 7
#define SQUARES_H 11
#define A4_W 210
#define A4_H 297
#define REAL_SQUARE_DIM 17.0f
#define REAL_OBJECT_DEFAULT_DIM 70.0f
#define SDF_RATIO 1.0f

class Hydrographic : public Scene
{
public:

	Hydrographic(const char* name) :
	Scene(name) {}

	virtual void Initialize(bool resetSimParams = true)
	{	
    // real dimensions of each model in mm
    float tetraDim = 60.0f;
    float sphereDim = 70.0f;
    float turtleDim = 96.0f;
    
    float sphereTexDim = 50.0f;
    float cunhaDim = 74.0f;
    float tembetaDim = 76.0f;
    float urnaDim = 94.0f;
    float oncaDim = 140.0f;

    //models without texture to chessboard test
		ModelData tetra = ModelData("../../data/teste/tetrahedron.obj", Vec3(0.0f, 0.0f, 0.01f), tetraDim, Vec3(0.0f, -90.0f, 0.0f));
    ModelData sphere = ModelData("../../data/teste/sphere.obj", Vec3(0.0f, 0.0f, 0.0f), sphereDim, Vec3(0.0f));
    ModelData turtle = ModelData("../../data/teste/tartaruga-centro_casco.obj", Vec3(0.09f, 0.0f, 0.025f), turtleDim, Vec3(0.0f, -185.6f, 0.0f));
    //models with texture to reverse-texture test
    ModelData sphereTex = ModelData("../../data/teste/Earth.obj", Vec3(0.0f, 0.0f, 0.0f), sphereTexDim, Vec3(0.0f, 0.0f, 0.0f));
    ModelData cunha = ModelData("../../data/teste/cunha-centro.obj", Vec3(0.0f, 0.0f, 0.0f), cunhaDim, Vec3(0.0f, 90.0f, -270.0f));
    ModelData tembeta = ModelData("../../data/old_map/tembeta-centro.obj", Vec3(0.0f, 0.0f, 0.0f), tembetaDim, Vec3(90.0f, 0.0f, 90.0f));
    ModelData urna = ModelData("../../data/teste/urna-centro.obj", Vec3(0.0f, 0.0f, 0.0f), urnaDim, Vec3(0.0f, 0.0f, 0.0f));
    ModelData onca = ModelData("../../data/teste/Onca_Poisson_15_16_0.obj", Vec3(0.0f, 0.0f, 0.0f), oncaDim, Vec3(0.0f, 0.0f, 0.0f));
    ModelData casco = ModelData("../../data/teste/tartaruga-centro_casco.obj", Vec3(0.1f, 0.0f, 0.0f), turtleDim, Vec3(0.0f, 0.0f, 180.0f));

    //ModelData turtle = ModelData("../../data/tartaruga_centro.obj", Vec3(0.09f, 0.0f, 0.1f), 0.125f, Vec3(0.0f, -95.5f, 0.0f)); //fator escala fixa medida 0.14
    //ModelData turtleRot = ModelData("tartaruga_centro.obj", Vec3(-0.07f, 0.0f, 0.2f), 0.130f, Vec3(0.0f, -95.53f, 0.0f));

		models.clear();
		//models without texture
    models.push_back(tetra);
		models.push_back(sphere);
		models.push_back(turtle);
    //models with texture
    models.push_back(sphereTex);
    models.push_back(cunha);
    models.push_back(tembeta);
    models.push_back(urna);
    models.push_back(onca);
    models.push_back(casco);

		//general params
    if (resetSimParams)
    {
      g_params.numIterations = 5; //5 iteracoes por default; se diminuir provoca maior esticamento
      g_numSubsteps = 1;
      // common params
      g_params.dynamicFriction = 1.0f;// coefs de friccao - altos valores substituem o recurso de ades�o
      g_params.staticFriction = 1.0;
      g_params.particleFriction = 1.0f;
      g_params.restitution = 0.0f; // coef. restituicao (colisao totalmente inelastica, corpos seguem juntos)
      g_params.adhesion = 0.08f; // coef de adesao
      g_params.sleepThreshold = 0.01f;

      //g_params.maxSpeed
      //g_params.maxAcceleration

      //g_params.shockPropagation = 0.3f;
      //g_params.dissipation = 0.25f;
      //g_params.damping = 0.5f;

      // cloth params
      g_params.drag = 1.8f;//0.3f;//3.5f;//força de arrasto ao tecido. aumenta sensa��o de elasticidade
      g_params.lift = 1.2f;//0.2f;//1.8f;// forca de sustentação
                           // collision params
      float radius = 0.012f;
      g_params.radius = radius;//0.012f;   //radius*1.0f; // raio max de intera��o entre part�culas
      g_params.collisionDistance = 0.012f;	//dist�ncia mantida entre a part�cula e o shape ap�s colis�o
      g_params.particleCollisionMargin = radius*0.5f;//
      g_params.shapeCollisionMargin = radius*0.5f;//
      g_params.numPlanes = 0;
      // relaxation solver params
      g_params.relaxationMode = eNvFlexRelaxationGlobal;
      g_params.relaxationFactor = 0.5f;
      //g_params.relaxationMode = eNvFlexRelaxationLocal;
      //g_params.relaxationFactor = 0.2f;
      // plastic params not used
      //g_params.plasticThreshold = 0.1f;
      //g_params.plasticCreep = 0.3f;
    }

    //select a model to load
    if (g_selectedModel >= 0 && g_selectedModel < models.size())
    {
      selectedModel = g_selectedModel;
    }
    else
    {
      //without texture
      //0: tetraedro 
      //1: sphere
      //2: turtle
      //with texture
      //3: sphere
      //4: cunha
      //5: tembeta
      //6: urna
      //7: onca
      //8: tartaruga
      selectedModel = 2;
    }

    Mesh* mesh = NULL; //mesh model to create sdf structures

    // used in batch tests
    if (g_meshStepTest)
    {
      //create sphere mesh parametrized by sectors
      sphereSectors = g_meshFactor;
      cout << "Selecting sphere mesh with " << sphereSectors << " segments and " << sphereSectors << " slices " << endl;
      mesh = CreateSphere(sphereSectors, sphereSectors, 1.0f);
    }
    else 
    {
      if (g_filmStepTest) 
      {
        factor = g_filmFactor;
      }
      if (g_voxelStepTest) 
      {
        voxelDim = g_voxelFactor;
      }
      //create mesh loading file
      mesh = ImportMesh(GetFilePathByPlatform(models[selectedModel].path).c_str());
    }

    cout << "Mesh positions:" << mesh->m_positions.size() << endl;
    cout << "Mesh triangles:" << mesh->m_indices.size()/3 << endl;

    //create sdf model
    sdfRotation = GetSdfRotation(models[selectedModel].rotation);
    sdfMesh = CreateRigidBodySDF(mesh, GetFilePathByPlatform(models[selectedModel].path).c_str(), createFile, voxelDim, sdfRotation, sdfCentroid, sdfMargin, expand);

    dimensionFactor = SDF_RATIO / voxelDim;
    g_centroid = dimensionFactor * sdfCentroid;

    sdfTranslationOffset = -Vec3(g_centroid.x, 0.0f, g_centroid.z) ; //only for sdf mesh
    initialPos = initialRigidPos + sdfTranslationOffset + models[selectedModel].translation;

    Vec3 pos = initialPos;
    Quat rot = QuatFromAxisAngle(Vec3(1.0f), 0.0f);

    AddSDF(sdfMesh, pos, rot, SDF_RATIO);
    meshIndex = g_buffers->shapePositions.size() - 1;

    // translate centroid to model matrix
    /*
    float dimensionFactor = 1.0f / voxelDim;
    g_centroid = Vec3(g_model * Vec4(dimensionFactor * sdfCentroid, 1.0f));
    printf("centroidPosition (%.2f, %.2f, %.2f)\n", g_centroid.x, g_centroid.y, g_centroid.z);
    Mat44 translationCenter = TranslationMatrix(Point3(g_centroid));
    Vec3 angle = Vec3(0.0f);
    float angleStep = 0.01f;
    
    Mat44 rotX = RotationMatrix(angle.x, Vec3(1.0f, 0.0f, 0.0f));
    Mat44 rotY = RotationMatrix(angle.y, Vec3(0.0f, 1.0f, 0.0f));
    Mat44 rotZ = RotationMatrix(angle.z, Vec3(0.0f, 0.0f, 1.0f));
    float minDiag = FLT_MAX;
    Vec3 minPositionDiag = FLT_MAX;
    Vec3 maxPositionDiag = -FLT_MAX;
    Vec3 minPositionAngle = Vec3(0.0f);

    for (; angle.x < 2 * M_PI; angle.x += angleStep)
    {
      rotX = RotationMatrix(angle.x, Vec3(1.0f, 0.0f, 0.0f));
      for (; angle.y < 2 * M_PI; angle.y += angleStep) 
      {
        rotY = RotationMatrix(angle.y, Vec3(0.0f, 1.0f, 0.0f));
        for (; angle.z < 2 * M_PI; angle.z += angleStep) {
          rotZ = RotationMatrix(angle.z, Vec3(0.0f, 0.0f, 1.0f));
          Mat44 rot = rotZ * rotY * rotX;
          
          Mat44 modelStep = translationCenter * rot;

          Vec3 minPosition, maxPosition;
          Vec3 positionAux = modelStep * Vec4(Vec3(mesh->m_positions[0]), 1.0f);
          minPosition = maxPosition = positionAux;
          for (int i = 0; i < mesh->m_positions.size(); i++) {
            Vec3 positionAux = modelStep * Vec4(Vec3(mesh->m_positions[i]), 1.0f);
            minPosition = Min(positionAux, minPosition);
            maxPosition = Max(positionAux, maxPosition);
          }

          float diag = Length(maxPosition - minPosition);
          if (diag < minDiag)
          {
            minDiag = diag;
            minPositionAngle = angle;
            minPositionDiag = minPosition;
            maxPositionDiag = maxPosition;
          }
        }
      }
    }
    */

    // free memory
    delete mesh;

    // define model matrix to start simulation
    g_model = TranslationMatrix(Point3(pos))*RotationMatrix(Quat(rot))*ScaleMatrix(1.0f);
    // gpu mesh created by assimp import optimized for loading a full texture atlas features and rendering
    g_gpu_rigid_mesh = CreateGpuMesh(GetFilePathByPlatform(models[selectedModel].path).c_str(), sdfRotation, sdfMargin);

    // create hydrographic film
    // parameters to create a film with M x N vertices with M an N proportinal to 7 x 11 (squares used in physical hydrographic)
    // parametrizing the M x N dimensions and the spacing between the vertices

    // factor = 1, creates a grid with 7 x 11 vertices
    // factor = 2, creates a grid with 14 x 22 and so on
    // independent of scale
    // this make it easy vary the resolution of film by changing only one parameter (film factor) 
    // and keeping the aspect ratio 7 x 11 constant proportional to an A4 sheet used in physical hydrographic
    g_filmDimX = factor * (SQUARES_W + 1);
    g_filmDimZ = factor * (SQUARES_H + 1);
    // spacing parameter applies the simulated dimension proportional to real dimensions
    // keeps the simulated film with squares proportional to physical film
    // considering simulated model vs the real model dimensions, provided by the user
    g_realDistanceFactor = GetScaleFactor(models[selectedModel].realSize, SDF_RATIO, sdfMargin);
    SimulatedModelScaledSquareSize = REAL_SQUARE_DIM * g_realDistanceFactor;

    float spacing = SimulatedModelScaledSquareSize / factor;

    GetShapeBounds(lower, upper);
    g_meshCenter = (lower + upper) * 0.5f;

    g_filmCenter = Vec3((g_filmDimX - 1)*spacing, gridY, (g_filmDimZ - 1)*spacing) * offsetCenterXZ;

    Matrix44 filmModel = TranslationMatrix(Point3(g_filmCenter));

    int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
    CreateHydrographicSpringGrid(g_filmCenter, g_meshCenter, g_filmDimX, g_filmDimZ, 1, spacing, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f, g_horizontalInvMass, g_verticalInvMass, false, false);

    // initialize contact structures for generate reverse texture mapping
    g_contact_positions.resize(g_buffers->positions.size());
    g_contact_normals.resize(g_buffers->normals.size());
    g_contact_indexes.resize(g_buffers->triangles.size());
    g_contact_uvs.resize(g_buffers->positions.size());
    g_compens_colors.resize(g_buffers->positions.size());

    // create a copy of flat film positinos before deformation occurs
    // do the same to normals and triangles array
    g_buffers->positions.copyto(&g_contact_positions[0], g_buffers->positions.size());
    for (int i = 0; i < g_buffers->positions.size(); i++)
    {
      g_contact_uvs[i] = Vec4(-1.0f); // initialize contact uv with -1 means this vertice has not contact with rigid body
      g_compens_colors[i] = Vec4(1.0f); // initial compens is neutral value
    }
    g_buffers->normals.copyto(&g_contact_normals[0], g_buffers->normals.size());
    g_buffers->triangles.copyto(&g_contact_indexes[0], g_buffers->triangles.size());
    
    // flex data is only handle the interaction between the soft bodies
    // handle the positions and collisions
    // data structure for render the scene and show texture mapping properly is provided by gpu_mesh structure created here
    // first time this structure is created and the buffers allocated here
    // this same structure is updated in the render procedure in the main loop, with the positions updated by flex
    g_gpu_film_mesh = CreateGpuFilm(filmModel, &g_buffers->positions[0], &g_buffers->normals[0], g_buffers->uvs.size() ? &g_buffers->uvs[0] : NULL, g_buffers->positions.size(), &g_buffers->triangles[0], g_buffers->triangles.size());

    //reset draw options
		g_drawSprings = false;
    g_generateContactsTexture = true;
    g_drawReverseTexture = false;
    g_drawFixedSeams = true;
    g_complete = false;
    g_pause = false;
    g_drawHydrographic = true;
    g_drawHydrographicCollisionMesh = true;
    //reset values
    g_fps = 0.0f;
    g_frame = 0;
    mTime = 0.0f;
	}

 	void Update()
	{
		mTime += g_dt;

		float time = Max(0.0f, mTime-startTime);
		float lastTime = Max(0.0f, time-g_dt);

    GetShapeBounds(lower, upper);
		// stop animation when hidrographics is complete and generate contacts texture
		if (upper.y + 0.8f < gridY && !g_complete)
		{
      g_complete = true;
      g_generateContactsTexture = true;
    }

    Vec3 pos = initialPos + g_dippingVelocity * time;
    Vec3 prevPos = initialPos + g_dippingVelocity * lastTime;

		Quat rot = QuatFromAxisAngle(Vec3(1.0f), 0.0f);
		Quat prevRot = QuatFromAxisAngle(Vec3(1.0f), 0.0f);
    
    // updates model matrix for rendering
    g_model = TranslationMatrix(Point3(pos))*RotationMatrix(Quat(rot))*ScaleMatrix(1.0f);
    // update centroid for render the centroid move
    // translate centroid to model matrix
    g_centroid = Vec3(g_model * Vec4(dimensionFactor * sdfCentroid, 1.0f));

		g_buffers->shapePositions[meshIndex] = Vec4(pos, 0.0f);
		g_buffers->shapeRotations[meshIndex] = rot;

		g_buffers->shapePrevPositions[meshIndex] = Vec4(prevPos, 0.0f);
		g_buffers->shapePrevRotations[meshIndex] = prevRot;

		UpdateShapes(); // update data to flex check colision in updated positions

  }

	void DoGui()
	{
		imguiLabel("Hydrographic options");
		if (imguiCheck("Create SDF file", createFile))
			createFile = !createFile;

    imguiSlider("Max distance UV", &g_max_distance_uv, 0.0f, 1.0f, 0.005f);
    imguiSlider("Texture epsilon", &g_near_distance_uv, 0.0f, 1.0f, 0.0001f);
    imguiSlider("Dipping velocity", &g_dippingVelocity.y, -0.5f, 0.5f, 0.005f);
		imguiSlider("Horizontal invMass", &g_horizontalInvMass, 0.0f, 1.0f, 0.005f);
		imguiSlider("Vertical invMass", &g_verticalInvMass, 0.0f, 1.0f, 0.005f);
    imguiSlider("Stretch Stiffness", &stretchStiffness, 0.0f, 1.0f, 0.0001f);

    if (imguiCheck("Draw Collision Mesh", g_drawHydrographicCollisionMesh))
      g_drawHydrographicCollisionMesh = !g_drawHydrographicCollisionMesh;
    if (imguiCheck("Draw Hydrographic Film", g_drawHydrographic))
      g_drawHydrographic = !g_drawHydrographic;

    if (imguiCheck("Draw Reverse Texture", g_drawReverseTexture))
    {
      g_drawReverseTexture = !g_drawReverseTexture;
      g_drawHydrographic = false;
      g_drawHydrographicCollisionMesh = false;      
      g_drawStretchColor = false;

      g_createReverseTextureFile = true;


      if (g_drawReverseTexture)
      {
        if (g_drawColorCompensation)
          g_textureMode = 2;
        else
          g_textureMode = 1;
      }
      else
      {
        if (g_drawColorCompensation)
        {
          g_textureMode = 3;
        }
      }
    }

    if (imguiCheck("Draw Color Compensation", g_drawColorCompensation))
    {
      g_drawColorCompensation = !g_drawColorCompensation;
      g_drawHydrographic = false;
      g_drawHydrographicCollisionMesh = false;
      g_drawStretchColor = false;

      if (g_drawReverseTexture)
      {
        if (g_drawColorCompensation)
          g_textureMode = 2;
        else
          g_textureMode = 1;
      }
      else 
      {
        g_textureMode = 3;
      }

    }

    if (imguiCheck("Draw Stretch Colors", g_drawStretchColor))
    {
      g_drawStretchColor = !g_drawStretchColor;
      g_drawHydrographic = false;
      g_drawHydrographicCollisionMesh = false;
      g_drawReverseTexture = false;
      g_drawColorCompensation = false;

      g_textureMode = 4;
    }

    /*
    if (imguiCheck("Draw Spring Stiffness", g_drawStiffness))
      g_drawStiffness = !g_drawStiffness;

    if (imguiCheck("Draw Spring Stretching", g_drawStretching))
      g_drawStretching = !g_drawStretching;
    */
    
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
        g_selectedModel = i;
        g_resetScene = true;
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
  Vec3 offsetCenterXZ = Vec3(-0.5f, 0.0f, -0.5f);
  Vec3 sdfTranslationOffset = Vec3(0.0f);
  float dimensionFactor = 1.0f;
  Vec3 sdfCentroid = Vec3(0.0f);
  Mat44 sdfRotation;
  //Rigid model params
	Vec3 lower, upper;
	int meshIndex = 0;
	int selectedModel = 0;
  Vec3 initialPos;
  Vec3 initialRigidPos = Vec3(0.0f, 1.5f, 0.0f);
	NvFlexDistanceFieldId sdfMesh = NULL;
  int sphereSectors = 160;
	//Film params
  float stretchStiffness = 1.0f;// default value: 1.0; more stretch -> closer to .0f
  float bendStiffness = 0.75f; // not used
  float shearStiffness = 0.5f; // not used
  float gridY = 0.3f; // film Y position
  int factor = 23;
  float SimulatedModelScaledSquareSize; //simulated scaled size of chessboard square considering the current model simulated with 1.0 square ratio
	std::vector<ModelData> models;
};
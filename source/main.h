#pragma once

#define SDL_CONTROLLER_BUTTON_LEFT_TRIGGER (SDL_CONTROLLER_BUTTON_MAX + 1)
#define SDL_CONTROLLER_BUTTON_RIGHT_TRIGGER (SDL_CONTROLLER_BUTTON_MAX + 2)

int GetKeyFromGameControllerButton(SDL_GameControllerButton button)
{
	switch (button)
	{
	case SDL_CONTROLLER_BUTTON_DPAD_UP: {	return SDLK_q;		} // -- camera translate up
	case SDL_CONTROLLER_BUTTON_DPAD_DOWN: {	return SDLK_z;		} // -- camera translate down
	case SDL_CONTROLLER_BUTTON_DPAD_LEFT: {	return SDLK_h;		} // -- hide GUI
	case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: {	return -1;			} // -- unassigned
	case SDL_CONTROLLER_BUTTON_START: {	return SDLK_RETURN;	} // -- start selected scene
	case SDL_CONTROLLER_BUTTON_BACK: {	return SDLK_ESCAPE;	} // -- quit
	case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: {	return SDLK_UP;		} // -- select prev scene
	case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: {	return SDLK_DOWN;	} // -- select next scene
	case SDL_CONTROLLER_BUTTON_A: {	return SDLK_g;		} // -- toggle gravity
	case SDL_CONTROLLER_BUTTON_B: {	return SDLK_p;		} // -- pause
	case SDL_CONTROLLER_BUTTON_X: {	return SDLK_r;		} // -- reset
	case SDL_CONTROLLER_BUTTON_Y: {	return SDLK_o;		} // -- step sim
	case SDL_CONTROLLER_BUTTON_RIGHT_TRIGGER: {	return SDLK_SPACE;	} // -- emit particles
	default: {	return -1;			} // -- nop
	};
};

//
// Gamepad thresholds taken from XINPUT API
//
#define XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE  7849
#define XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE 8689
#define XINPUT_GAMEPAD_TRIGGER_THRESHOLD    30

int deadzones[3] = { XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE, XINPUT_GAMEPAD_TRIGGER_THRESHOLD };

inline float joyAxisFilter(int value, int stick)
{
	//clamp values in deadzone to zero, and remap rest of range so that it linearly rises in value from edge of deadzone toward max value.
	if (value < -deadzones[stick])
		return (value + deadzones[stick]) / (32768.0f - deadzones[stick]);
	else if (value > deadzones[stick])
		return (value - deadzones[stick]) / (32768.0f - deadzones[stick]);
	else
		return 0.0f;
}

SDL_GameController* g_gamecontroller = NULL;

// main functions
// simulation
void Init(int scene, bool centerCamera);
void RandInit();
void Reset();
void Shutdown();
void UpdateCamera();
void SyncScene();
void UpdateScene();
void RenderDebug();
void StoreReport(int type, int factor);
void ShowReport();
void UpdateFrame(bool &quit);
void ErrorCallback(NvFlexErrorSeverity, const char* msg, const char* file, int line);
// imGui
int DoUI();
// user interaction 
int GetKeyFromGameControllerButton(SDL_GameControllerButton button);
inline float joyAxisFilter(int value, int stick);
void ReshapeWindow(int width, int height);
void InputArrowKeysUp(int key, int x, int y);
bool InputKeyboardDown(unsigned char key, int x, int y);
void InputKeyboardUp(unsigned char key, int x, int y);
void MouseFunc(int b, int state, int x, int y);
void MousePassiveMotionFunc(int x, int y);
void MouseMotionFunc(unsigned state, int x, int y);
void ControllerButtonEvent(SDL_ControllerButtonEvent event);
void ControllerDeviceUpdate();
void SDLInit(const char* title);
void SDLMainLoop();

// simulation buffers
struct SimBuffers
{
	// particles
	NvFlexVector<Vec4> positions;
	NvFlexVector<Vec4> restPositions;
	NvFlexVector<Vec3> velocities;
	NvFlexVector<int> phases;
	NvFlexVector<Vec4> normals;
	NvFlexVector<int> activeIndices;
	NvFlexVector<int> diffuseCount;

	NvFlexVector<Vec4>  originalPositions;	// for compute displacement heatmap


	// convexes
	NvFlexVector<NvFlexCollisionGeometry> shapeGeometry;
	NvFlexVector<Vec4> shapePositions;
	NvFlexVector<Quat> shapeRotations;
	NvFlexVector<Vec4> shapePrevPositions;
	NvFlexVector<Quat> shapePrevRotations;
	NvFlexVector<int> shapeFlags;
	// springs
	NvFlexVector<int> springIndices;
	NvFlexVector<float> springLengths;
	NvFlexVector<float> springStiffness;
	// triangles
	NvFlexVector<int> triangles;
	NvFlexVector<Vec3> triangleNormals;
	NvFlexVector<Vec4> uvs;

	SimBuffers(NvFlexLibrary* l) :
		positions(l), restPositions(l), velocities(l), phases(l), normals(l), activeIndices(l), diffuseCount(l),
		originalPositions(l),
		shapeGeometry(l), shapePositions(l), shapeRotations(l),
		shapePrevPositions(l), shapePrevRotations(l), shapeFlags(l),
		springIndices(l), springLengths(l), springStiffness(l),
		triangles(l), triangleNormals(l), uvs(l)
	{}
};

SimBuffers* g_buffers;


#ifdef TRACK_DISPLACEMENTS
SimBuffers* g_displacement_buffers;
#endif

void MapBuffers(SimBuffers* buffers)
{
	// particles
	buffers->positions.map();
	buffers->restPositions.map();
	buffers->velocities.map();
	buffers->phases.map();
	buffers->normals.map();
	buffers->activeIndices.map();
	buffers->diffuseCount.map();

	buffers->originalPositions.map();

	// convexes
	buffers->shapeGeometry.map();
	buffers->shapePositions.map();
	buffers->shapeRotations.map();
	buffers->shapePrevPositions.map();
	buffers->shapePrevRotations.map();
	buffers->shapeFlags.map();
	// springs
	buffers->springIndices.map();
	buffers->springLengths.map();
	buffers->springStiffness.map();
	// triangles
	buffers->triangles.map();
	buffers->triangleNormals.map();
	buffers->uvs.map();
}

void UnmapBuffers(SimBuffers* buffers)
{
	// particles
	buffers->positions.unmap();
	buffers->restPositions.unmap();
	buffers->velocities.unmap();
	buffers->phases.unmap();
	buffers->normals.unmap();
	buffers->activeIndices.unmap();
	buffers->diffuseCount.unmap();

	buffers->originalPositions.unmap();
	
	// convexes
	buffers->shapeGeometry.unmap();
	buffers->shapePositions.unmap();
	buffers->shapeRotations.unmap();
	buffers->shapePrevPositions.unmap();
	buffers->shapePrevRotations.unmap();
	buffers->shapeFlags.unmap();
	// springs
	buffers->springIndices.unmap();
	buffers->springLengths.unmap();
	buffers->springStiffness.unmap();
	// triangles
	buffers->triangles.unmap();
	buffers->triangleNormals.unmap();
	buffers->uvs.unmap();
}

SimBuffers* AllocBuffers(NvFlexLibrary* lib)
{
	return new SimBuffers(lib);
}

void DestroyBuffers(SimBuffers* buffers)
{
	// particles
	buffers->positions.destroy();
	buffers->restPositions.destroy();
	buffers->velocities.destroy();
	buffers->phases.destroy();
	buffers->normals.destroy();
	buffers->activeIndices.destroy();
	buffers->diffuseCount.destroy();

	buffers->originalPositions.destroy();
	
	// convexes
	buffers->shapeGeometry.destroy();
	buffers->shapePositions.destroy();
	buffers->shapeRotations.destroy();
	buffers->shapePrevPositions.destroy();
	buffers->shapePrevRotations.destroy();
	buffers->shapeFlags.destroy();
	// springs
	buffers->springIndices.destroy();
	buffers->springLengths.destroy();
	buffers->springStiffness.destroy();
	// triangles
	buffers->triangles.destroy();
	buffers->triangleNormals.destroy();
	buffers->uvs.destroy();

	delete buffers;
}

using namespace std;

SDL_Window* g_window;			// window handle
unsigned int g_windowId;	// window id

int g_screenWidth = 1280;
int g_screenHeight = 720;
int g_msaaSamples = 8;

int g_numSubsteps;

// a setting of -1 means Flex will use the device specified in the NVIDIA control panel
int g_device = -1;
char g_deviceName[256];
bool g_vsync = true;

// para os testes
bool g_filmStepTest = false;
bool g_meshStepTest = false;
bool g_voxelStepTest = false;
int  g_selectedModel = -1;

int g_filmFactor = 1;
int g_filmFactorMin = 1;
int g_filmFactorMax = 50;
int g_filmFactorStep = 5;

int g_meshFactor = 10;
int g_meshFactorMin = 10;
int g_meshFactorMax = 400;
int g_meshFactorStep = 10;

int g_voxelFactor = 64;
int g_voxelactorMin = 64;
int g_voxelFactorMax = 320;
int g_voxelFactorStep = 16;

int g_meshVertices;

// computes fps
float g_fps = 0.0f;
float g_avgFPS = 0.0f;
float g_avgUpdateTime = 0.0f;
float g_avgRenderTime = 0.0f;
float g_avgWaitTime = 0.0f;
float g_avgLatencyTime = 0.0f;

bool g_benchmark = false;
bool g_extensions = true;
bool g_teamCity = false;
bool g_interop = true;
bool g_d3d12 = false;
bool g_useAsyncCompute = true;
bool g_increaseGfxLoadForAsyncComputeTesting = false;
int g_graphics = 0;	// 0=ogl, 1=DX11, 2=DX12

FluidRenderer* g_fluidRenderer;
FluidRenderBuffers* g_fluidRenderBuffers;
DiffuseRenderBuffers* g_diffuseRenderBuffers;


NvFlexSolver* g_solver;
NvFlexSolverDesc g_solverDesc;
NvFlexLibrary* g_flexLib;

#ifdef TRACK_DISPLACEMENTS
// displacement solver
NvFlexSolver* g_displacements_solver;
// hydrographics
vector<float> displacements;		// for compute hydrographic distortion
									//vector<Vec3>  displacedPositions;	// for compute hydrographic distortion
									//vector<bool>  tracking;
									//std::map<int, float> frameTracking;
									//bool g_tracking;
#endif
NvFlexParams g_params;
NvFlexTimers g_timers;
int g_numDetailTimers;
NvFlexDetailTimer * g_detailTimers;

int g_maxDiffuseParticles;
unsigned char g_maxNeighborsPerParticle;
int g_numExtraParticles;
int g_numExtraMultiplier = 1;
int g_maxContactsPerParticle;

// mesh used for deformable object rendering
string g_basePath;
Mesh* g_mesh;
GpuMesh* g_film_mesh;
GpuMesh* g_gpu_mesh;

vector<Point3> g_meshRestPositions;
const int g_numSkinWeights = 4;

vector<float> frametimes;
vector<float> update_times;
vector<float> render_times;
vector<float> wait_times;
vector<float> latency_times;

struct TestReport {
	int type;
	int factor;
	int springs;
	float avgFPS;
	float avgUpdateTime;
	float avgRenderTime;
	float avgWaitTime;
	float avgLatencyTime;
	int voxel;
	int meshVertices;
};

vector<TestReport> report;

// mapping of collision mesh to render mesh
std::map<NvFlexTriangleMeshId, GpuMesh*> g_meshes;
//std::map<NvFlexDistanceFieldId, GpuMesh*> g_fields;
NvFlexDistanceFieldId g_sdf_mesh;
//Mesh* g_mesh_debug = NULL;

/* Note that this array of colors is altered by demo code, and is also read from global by graphics API impls */
Colour g_colors[] =
{
	Colour(0.0f, 0.5f, 1.0f),
	Colour(0.797f, 0.354f, 0.000f),
	Colour(0.092f, 0.465f, 0.820f),
	Colour(0.000f, 0.349f, 0.173f),
	Colour(0.875f, 0.782f, 0.051f),
	Colour(0.000f, 0.170f, 0.453f),
	Colour(0.673f, 0.111f, 0.000f),
	Colour(0.612f, 0.194f, 0.394f)
};

// flag to request collision shapes be updated
bool g_shapesChanged = false;
float translateX = 0.0f;
float translateZ = 0.0f;
Vec3 gridPosition;
int gridDimX = 0;
int gridDimZ = 0;
float gridSpacing = 0.0f;
unsigned int texHeatmapId;
float dippingVelocity = -0.3f;
float horizontalInvMass = 0.9f;
float verticalInvMass = 0.9f;

std::vector<Vec3> g_camPos;
std::vector<Vec3> g_camAngle;
int g_camIndex = 0;
Vec3 g_camVel(0.0f);
Vec3 g_camSmoothVel(0.0f);
Vector3 g_meshCenter = Vector3(0.0f);
Mat44 g_view, g_proj;

float g_camSpeed;
float g_camNear;
float g_camFar;

Vec3 g_lightPos;
Vec3 g_lightDir = Normalize(Vec3(5.0f, -15.0f, 7.5f));
//Vec3 g_lightTarget;
//bool g_centerLight = true;

bool g_pause = false;
bool g_step = false;
bool g_capture = false;
bool g_showHelp = true;
bool g_tweakPanel = true;
bool g_fullscreen = false;
bool g_wireframe = false;
bool g_debug = false;

bool g_emit = false;
bool g_warmup = false;

float g_waveFloorTilt = 0.0f;

Vec3 g_sceneLower;
Vec3 g_sceneUpper;

float g_blur;
float g_ior;
bool g_drawEllipsoids;
bool g_drawPoints;
bool g_drawMesh;
bool g_drawCloth;
bool g_drawHydrographic;
bool g_drawHydrographicMesh = true;
bool g_drawHydrographicCollisionMesh = true;
bool g_drawAABB = false;
Mesh* g_mesh_rigid;
bool g_drawShadows;
float g_expandCloth;	// amount to expand cloth along normal (to account for particle radius)

bool g_drawOpaque;
int g_drawSprings;		// 0: no draw, 1: draw stretch 2: draw tether
bool g_drawBases = false;
bool g_drawDisplacements = true;
bool g_drawContacts = false;
bool g_drawNeighbors = false;
bool g_drawAxis = false;
bool g_drawNormals = false;
bool g_drawDiffuse;
bool g_drawShapeGrid = false;
bool g_drawDensity = false;
float g_pointScale;
float g_ropeScale;
float g_drawPlaneBias;	// move planes along their normal for rendering

float g_diffuseScale;
float g_diffuseMotionScale;
bool g_diffuseShadow;
float g_diffuseInscatter;
float g_diffuseOutscatter;

float g_dt = 1.0f / 60.0f;	// the time delta used for simulation
float g_realdt;				// the real world time delta between updates

float g_waitTime = 0.0f;		// the CPU time spent waiting for the GPU
float g_updateTime = 0.0f;     // the CPU time spent on Flex
float g_renderTime = 0.0f;		// the CPU time spent calling OpenGL to render the scene
								// the above times don't include waiting for vsync
float g_simLatency = 0.0f;     // the time the GPU spent between the first and last NvFlexUpdateSolver() operation. Because some GPUs context switch, this can include graphics time.

int g_scene = 0;
int g_selectedScene = g_scene;
int g_levelScroll;			// offset for level selection scroll area
bool g_resetScene = false;  //if the user clicks the reset button or presses the reset key this is set to true;

int g_frame = -1;
int g_numSolidParticles = 0;

int g_mouseParticle = -1;
float g_mouseT = 0.0f;
Vec3 g_mousePos;
float g_mouseMass;
bool g_mousePicked = false;

// mouse
int g_lastx;
int g_lasty;
int g_lastb = -1;

bool g_profile = false;
bool g_outputAllFrameTimes = false;
bool g_asyncComputeBenchmark = false;

ShadowMap* g_shadowMap;

//Vec4 g_fluidColor;
Vec4 g_diffuseColor;
Vec3 g_meshColor;
Vec3  g_clearColor;
float g_lightDistance;
float g_fogDistance;

FILE* g_ffmpeg;
FILE* g_film;

class Scene;
vector<Scene*> g_scenes;

inline float sqr(float x) { return x*x; }
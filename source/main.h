#pragma once
#define CAM_DISTANCE_R 93.7f //163.02f
#define IMG_WIDTH 1082

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
void Init(int scene, bool centerCamera, bool resetSimParams);
void RandInit();
void Reset();
void Destroy();
void Shutdown();
void UpdateCamera();
void SyncScene();
void UpdateScene();
void RenderDebug();
void BuildReverseTextureMapping();
void PostProcessReverseTexture();
void StoreReport(int type, int factor);
void ShowReport();
bool isTestingMode();
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
		positions(l), restPositions(l), velocities(l), phases(l), normals(l), activeIndices(l), 
		shapeGeometry(l), shapePositions(l), shapeRotations(l),
		shapePrevPositions(l), shapePrevRotations(l), shapeFlags(l),
		springIndices(l), springLengths(l), springStiffness(l),
		triangles(l), triangleNormals(l), uvs(l)
	{}
};

SimBuffers* g_buffers;

void MapBuffers(SimBuffers* buffers)
{
	// particles
	buffers->positions.map();
	buffers->restPositions.map();
	buffers->velocities.map();
	buffers->phases.map();
	buffers->normals.map();
	buffers->activeIndices.map();
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
float g_aspect = float(g_screenWidth) / g_screenHeight;

int g_msaaSamples = 8;

int g_numSubsteps;
float g_numSubstepsAux;
float g_numIterationsAux;
// render phong parameters
Vec3 g_meshColor;
Vec3  g_clearColor;
float g_lightDistance;
Vec4 g_lightColor = Vec4(1.0f, 1.0f, 1.0f, 1.0f);
Vec4 g_ambientColor = Vec4(0.15f, 0.15f, 0.15f, 1.0f);
Vec4 g_diffuseColor = Vec4(1.0f, 1.0f, 1.0f, 1.0f);
Vec4 g_specularColor = Vec4(1.0f, 1.0f, 1.0f, 1.0f);
unsigned int g_specularExpoent = 40;

// a setting of -1 means Flex will use the device specified in the NVIDIA control panel
int g_device = -1;
char g_deviceName[256];
bool g_vsync = false; // 0 for immediate updates, 1 for updates synchronized with the vertical retrace.

// for tests
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
int g_voxelFactorMax = 400;
float g_voxelFactorStep = 1.226; //multiplier factor to a geometric projection from 64 to 400.5
                                 //depending on memory

int g_meshVertices;

// computing fps
float g_fps = 0.0f;
float g_avgFPS = 0.0f;
float g_avgUpdateTime = 0.0f;
float g_avgRenderTime = 0.0f;
float g_avgWaitTime = 0.0f;
float g_avgLatencyTime = 0.0f;

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

bool g_benchmark = false;
bool g_extensions = true;
bool g_teamCity = false;
bool g_interop = true;
bool g_d3d12 = false;
bool g_useAsyncCompute = true;
bool g_increaseGfxLoadForAsyncComputeTesting = false;
int g_graphics = 0;	// 0=ogl, 1=DX11, 2=DX12

NvFlexSolver* g_solver;
NvFlexSolverDesc g_solverDesc;
NvFlexLibrary* g_flexLib;

// store data of flat distorted film
std::vector<Vec4> g_contact_positions;
std::vector<Vec4> g_contact_normals;
std::vector<Vec4> g_contact_uvs;
std::vector<Vec4> g_stretch_colors;
std::vector<Vec4> g_compens_colors;
std::vector<int> g_contact_indexes;
float g_max_distance_uv = 0.36f;
float g_near_distance_uv = 0.001f;
float g_weight1 = 0.0f;
float g_weight2 = 1.0f;
float g_tesselation_inner = 1.0f;
float g_tesselation_outer = 2.0f;

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

GpuMesh* g_gpu_film_mesh;
GpuMesh* g_gpu_rigid_mesh;

// mapping of collision mesh to render mesh
std::map<NvFlexTriangleMeshId, GpuMesh*> g_meshes;
NvFlexDistanceFieldId g_sdf_mesh;

bool g_shapesChanged = false; // flag to request collision shapes be updated
Vec3 g_filmCenter;
int g_filmDimX = 0;
int g_filmDimZ = 0;
Vec3 g_dippingVelocity = Vec3(0.0f, -0.3f, 0.0f);
float g_horizontalInvMass = 1.0f;
float g_verticalInvMass = 1.0f;
std::vector<Vec3> g_camPos;
std::vector<Vec3> g_camAngle;
int g_camIndex = 0;
int g_camIndexAux;
Vec3 g_camVel(0.0f);
Vec3 g_camSmoothVel(0.0f);
float g_camDistance;
float g_realDipping = 0.0f;
float g_realDistanceFactor;
Vector3 g_meshCenter = Vector3(0.0f);
Mat44 g_model, g_view, g_proj;
Vec3 g_centroid;

// camera parameters
float dx = 1307.0f;
float dy = 925.0f;
// 
float dX = 297.0f;
float dY = 210.0f;
float dZ = 71.0f;
//

float fx = (dx / dX)*dZ;
float fy = (dy / dY)*dZ;

//float g_fov = 2 * float(atan(0.5*g_screenHeight / fy)); // *180 / M_PI;
float g_fov = kPi / 4.0f;
float g_camSpeed;
float g_camNear = .001f;
float g_camFar = 100.0f;

Vec3 g_lightPos;
Vec3 g_lightDir = Normalize(Vec3(5.0f, -15.0f, 7.5f));

bool g_pause = false;
bool g_complete = false;
bool g_step = false;
bool g_capture = false;
bool g_showHelp = true;
bool g_tweakPanel = true;
bool g_fullscreen = false;
bool g_wireframe = false;
bool g_debug = false;

bool g_emit = false;
bool g_warmup = false;

Vec3 g_sceneLower;
Vec3 g_sceneUpper;

bool g_drawHydrographic = true;
bool g_drawHydrographicCollisionMesh = true;
bool g_drawStiffness = false;
bool g_drawStretching = false;
bool g_drawStretchColor = false;
int g_textureMode = 1;
int g_drawSprings;		// 0: no draw, 1: draw stretch 2: draw tether
bool g_drawBases = false;
bool g_drawReverseTexture = false;
bool g_drawColorCompensation = false;
bool g_createReverseTextureFile = false;
bool g_drawFixedSeams = false;
bool g_drawContacts = false;
bool g_generateContactsTexture = false;
bool g_drawNeighbors = false;
bool g_drawAxis = false;
bool g_drawNormals = false;
bool g_drawDiffuse;

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
bool g_resetSimParams = true;
int g_frame = -1;

// mouse
int g_lastx;
int g_lasty;
int g_lastb = -1;

bool g_profile = false;
bool g_outputAllFrameTimes = false;
bool g_asyncComputeBenchmark = false;

FILE* g_ffmpeg;
FILE* g_film;

class Scene;
vector<Scene*> g_scenes;

inline float sqr(float x) { return x*x; }
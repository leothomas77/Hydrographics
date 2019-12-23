#include <cuda.h>
#include <cuda_runtime_api.h>
#include <vector>
#include <limits>
#include <algorithm>

#include "../../core/core.h"
#include "../../core/maths.h"

#define CudaCheck(x) { cudaError_t err = x; if (err != cudaSuccess) { printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); assert(0); } }

static const int kNumThreadsPerBlock = 256;




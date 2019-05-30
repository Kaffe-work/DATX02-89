#include "kernel.h"
#include <stdint.h>
#include "boid.h"
#include "cub/cub.cuh"
#include "cub/device/device_radix_sort.cuh"

#include <stdio.h>
#include <algorithm>
#include "cub/util_allocator.cuh"
#include "cub/device/device_radix_sort.cuh"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "math.h"

// includes, cuda
#include <windows.h>
//#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

// Enable naive version here
// #define NAIVE_VERSION

// Enable timing here
//#define TIMING

// Use coordinate concatentation instead of Z-order
// #define BIT_CONCATENATION

/*
Compile on Linux machines in NC:
/chalmers/sw/sup64/cuda_toolkit-9.0.176.4/bin/nvcc --dont-use-profile -ldir /chalmers/sw/sup64/cuda_toolkit-9.0.176.4/nvvm/libdevice/ -I /chalmers/sw/sup64/cuda_toolkit-9.0.176.4/include -m64 -L /chalmers/sw/sup64/cuda_toolkit-9.0.176.4/lib64 ~/kernel.cu

You might need this before compiling:
PATH=$PATH:/chalmers/sw/sup64/cuda_toolkit-9.0.176.4/nvvm/bin
PATH=$PATH:/chalmers/sw/sup64/cuda_toolkit-9.0.176.4/bin

Compile on Windows 10 machine with CUDA 9.1 toolkit available:
nvcc kernel.cu -c -o kernel_win -use_fast_math -m64 -arch=sm_61 -Xcudafe --diag_suppress=2865 -ccbin [path to your cl.exe]
This will produce an .obj file that can be included in your project along with kernel.h
Path to cl.exe is probably something like "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\bin\cl.exe"
If you use Visual Studio 2017 You might need to install "vc++ 14.0" addon for it to work

*/

/*
MAJOR TODO: Replacing glm datatypes with CUDA's can improve performance,
for example vec3 -> float3, distance() -> norm3df()
*/



struct ObstaclePlane {
	glm::vec3 point, normal;

	ObstaclePlane(float p1, float p2, float p3, float n1, float n2, float n3)
		: point(p1, p2, p3), normal(n1, n2, n3) {}
};

// Array for the walls
ObstaclePlane* walls = NULL;

/* A useful macro for displaying CUDA errors */
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = false)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

using namespace cub;

extern glm::vec3 cameraDir;
extern glm::vec3 cameraPos;
extern bool isLaserActive;

// These arrays hold the boids
extern Boid* boids;
Boid* boidsAlt = NULL;

// These arrays hold the (Z-order/morton encoded) cell ids
uint64_t* cellIDs = NULL;
uint64_t* cellIDsAlt = NULL;

// Array with all the boids. boidsAlt is a alternate array needed for the radixSort
int* boidIDs = NULL;
int* boidIDsAlt = NULL;

// Doublebuffers containing boidIDs and cellIDs, these are used by the radix sort function
DoubleBuffer<uint64_t> cellIDsBuf;
DoubleBuffer<int> boidIDsBuf;

// Calculate the maximum value of Morton encoded (Z-ordered) cell ids
#define shiftBitK(x, k) (int) ((x&(1<<k)) << k*2+2 | (x&(1<<k)) << k*2+1 | (x&(1<<k)) << k*2)
const int MAX_CELL_INDEX = (int)MAX_COORD / CELL_SIZE;
// number of bits needed to represent the maximum coordinate
// WARNING: this is dependent on MAX_COORDINATE and should ideally be computed at compile time with log2(MAX_COORDINATE) + 1
#define MIN_SHIFT 7 
#ifdef BIT_CONCATENATION
const int NR_CELLS = MAX_CELL_INDEX << MIN_SHIFT * 2 | MAX_CELL_INDEX << MIN_SHIFT | MAX_CELL_INDEX;
#else
const int NR_CELLS = shiftBitK(MAX_CELL_INDEX, 10)
| shiftBitK(MAX_CELL_INDEX, 9)
| shiftBitK(MAX_CELL_INDEX, 8)
| shiftBitK(MAX_CELL_INDEX, 7)
| shiftBitK(MAX_CELL_INDEX, 6)
| shiftBitK(MAX_CELL_INDEX, 5)
| shiftBitK(MAX_CELL_INDEX, 5)
| shiftBitK(MAX_CELL_INDEX, 4)
| shiftBitK(MAX_CELL_INDEX, 3)
| shiftBitK(MAX_CELL_INDEX, 2)
| shiftBitK(MAX_CELL_INDEX, 1)
| shiftBitK(MAX_CELL_INDEX, 0);
#endif


// These parameters are used by the CUDA functions
int blockSize = 256;
int numBlocksBoids = (NR_BOIDS + blockSize - 1) / blockSize;
int numBlocksCells = (NR_CELLS + blockSize - 1) / blockSize;

// Time variable
float t = 0.0f;

// A tempory storage for new velocities allows parallel processing of the boids velocities 
glm::vec3* newVelocities;

// These arrays hold the start and end indices for each cell which contains boids
int* cellStartIndex;
int* cellEndIndex;

// Get the cell based on the boids position
// TODO: floorf() on the individual coordinates might be faster!?
inline __device__ glm::vec3 getCell(glm::vec3 pos) {
	return glm::floor(pos * (1.0f / CELL_SIZE));
}

// The boids are given an ID, which is the index it has in the intial boid array
__global__ void initBoidIDs(int BoidIDs[], int nrBoids) {
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;
	if (i >= nrBoids) return;
	BoidIDs[i] = i;
}

// Spreads out bits with two zeroes in between
// TODO: find the source and give credit
__device__ uint64_t spreadOutByThree(uint64_t i) {
	i = (i | (i << 16)) & 0x030000FF;
	i = (i | (i << 8)) & 0x0300F00F;
	i = (i | (i << 4)) & 0x030C30C3;
	i = (i | (i << 2)) & 0x09249249;
	return i;
}


// Hash cell coords to morton code with "magic numbers"
__device__ uint64_t bitInterlaceMagic(int x, int y, int z) {
	return spreadOutByThree((uint64_t)x) | (spreadOutByThree((uint64_t)y) << 1) | (spreadOutByThree((uint64_t)z) << 2);
}

// Hash cell coords to morton code with simple bit concatenation
__device__ uint64_t bitConcatenation(int x, int y, int z) {
	return ((uint64_t)x & 0x1FFFFF) << (MIN_SHIFT * 2) | ((uint64_t)y & 0x1FFFFF) << MIN_SHIFT | ((uint64_t)y & 0x1FFFFF);
}

// This function is used when scanning the sorted boids cell-ids to see were cells starts and ends 
// dummyHostIndex is only for debugging on machines without device enabled
__global__ void detectCellIndexChange(int cellStarts[], int cellEnds[], uint64_t cellIDs[], int nrBoids) {
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;
	if (i >= nrBoids) return;

	int cellID = cellIDs[i];
	int nextCellID = cellIDs[i + 1];

	// TODO: determine if these if/else statements causes thread branching -> worse performance
	if (i == 0) {
		// This is the case for the first element in the boid array 
		cellStarts[cellID] = i;
	}
	else if (i == nrBoids - 1) {
		// This is the case for the last element in the boid array
		cellEnds[cellID] = i;
		return;
	}
	if (cellID != nextCellID) {
		// A change in cell index was detected!
		cellStarts[nextCellID] = i + 1;
		cellEnds[cellID] = i;
	}
}

__device__ inline bool isNonValid(glm::vec3 v) {
	return isinf(v.x) || isinf(v.y) || isinf(v.z) || isnan(v.x) || isnan(v.y) || isnan(v.z);
}


__device__ inline void tapAcceleration(float &acc, glm::vec3 force, glm::vec3 &newVel) {
	float magnitude = fminf(glm::length(force), acc);
	if (magnitude > 0.f) {
		newVel += magnitude * normalize(force);
	}
	acc -= magnitude;
}


// Update boid with index n
// TODO: This function can probably be shorted, but it can wait since it's not updated with the most recent features anyway
// WARNING! VERY MUCH TODO: RIGHT NOW IT CHECKS OUTSIDE WORLD BOUNDARIES FOR BOIDS
// IN EDGE CELLS, THIS WILL CAUSE ARRAY OUT OF-BOUNDS EXCEPTIONS
// WARNING! VERY MUCH TODO: RIGHT NOW IT CHECKS OUTSIDE WORLD BOUNDARIES FOR BOIDS
// IN EDGE CELLS, THIS WILL CAUSE ARRAY OUT OF-BOUNDS EXCEPTIONS

__global__ void computeVelocities(Boid boids[], int cellStarts[], int cellEnds[], uint64_t cellIDs[]
	, int nrBoids, Boid boidsUpdated[], ObstaclePlane walls[], glm::vec3 cameraPos, glm::vec3 cameraDir, bool isLaserActive) {

	int idx = threadIdx.x + (blockIdx.x * blockDim.x);
	if (idx >= nrBoids) return;

	float nrPrey = 0;
	float nrPredators = 0;

	Boid b = boids[idx]; // current boid whose neighbours we're checking
						 // initialize default values for each rule
						 // Decide which cell current boid is in
	glm::vec3 cell = getCell(b.position);

	float bIsPredator = (bool)(b.status & PREDATOR_FLAG);
	float bIsPrey = (bool)!(b.status & PREDATOR_FLAG);

	glm::vec3 oldVel = b.velocity;

	glm::vec3 avgPreyPos = glm::vec3(0);
	glm::vec3 avgPreyVel = b.velocity;
	glm::vec3 avgPredatorPos = glm::vec3(0);

	glm::vec3 separation = glm::vec3(0);
	glm::vec3 cohesion = glm::vec3(0);
	glm::vec3 alignment = glm::vec3(0);
	glm::vec3 avoidance = glm::vec3(0);

	glm::vec3 newVel = glm::vec3(0);

	bool preyFound = false;

	glm::vec3 closestPreyPos = glm::vec3(0.0);
	float epsilon = 0.f;

	// Start checking all 27 neighbouring cells
	// TODO: find out a clever way to iterate over cells in order of the morton code to get 
	// more coherent memory accesses
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			for (int k = -1; k <= 1; k++) {

#ifdef BIT_CONCATENATION
				uint64_t cellID = bitConcatenation((int)cell.x + i, (int)cell.y + j, (int)cell.z + k);
#else
				// calculate the (Morton encoded/Z-order) cell id based on cell coordinates
				uint64_t cellID = bitInterlaceMagic((int)cell.x + i, (int)cell.y + j, (int)cell.z + k);
#endif
				// TODO: decide wether the if-statement below causes thread branching 
				if (cellStarts[cellID] == -1) {
					continue; // cell is empty if it's start index is unchanged since last reset
				}
				// Iterate over all boids in neighbouring cell
				for (int l = cellStarts[cellID]; l <= cellEnds[cellID]; l++) {
					Boid n = boids[l];

					glm::vec3 delta = n.position - b.position;
					float distance = glm::length(delta);

					// exclude neighbours that are outside boid's scope 
					float validNeighbour = (distance > epsilon && distance < BOID_SCOPE);
					float nIsPredator = validNeighbour * (bool)(n.status & PREDATOR_FLAG);
					float nIsPrey = validNeighbour * (!(bool)(n.status & PREDATOR_FLAG));

					nrPrey += nIsPrey;
					nrPredators += nIsPredator;

					avgPreyPos += nIsPrey * n.position;
					avgPreyVel += nIsPrey * n.velocity;
					avgPredatorPos += nIsPredator * n.position;

					// PREY BEHAVIOR WEIGHTED AVERAGE
					// Separation and predator avoidance
					/*
					if(validNeighbour > 0){
					separation -= bIsPrey * nIsPrey * glm::normalize(delta) * SEPARATION_FACTOR;
					separation -= nIsPredator * glm::normalize(delta) * AVOID_FACTOR;
					}
					*/

					// PREY BEHAVIOR ARBITRATE
					if (validNeighbour > 0) {
						separation -= (bIsPrey * nIsPrey) * delta / (distance*distance);
						// not sure about dividing with square root of distance...
						avoidance -= (nIsPredator) * delta / sqrt(distance);
					}

					// Hunt 
					if (nIsPrey > 0 && (distance < glm::distance(b.position, closestPreyPos) || !preyFound)) {
						closestPreyPos = n.position;
						preyFound = true;
					}
				}
			}
		}
	}

	// ARBITRATE

	glm::vec3 repellentForce = glm::vec3(0.f);

	if (isLaserActive) {
		glm::vec3 point = cameraPos + dot(b.position - cameraPos, cameraDir) * cameraDir;
		float distance = glm::distance(b.position, point);
		repellentForce = (float)(distance < 30.f) * normalize(b.position - point) / distance;
	}

	//Avoid walls
	glm::vec3 wallAvoidance = glm::vec3(0.f);
	for (int m = 0; m < 6; m++) {
		ObstaclePlane o = walls[m];

		// Make a vector from plane orig point to the boid
		glm::vec3 v = b.position - o.point;
		// Take the dot product of that vector with the unit normal vector n
		// inverse distance from plane
		float dist = glm::dot(v, o.normal);
		float inverse_dist = 1.f / dist;
		// square the inverse (power law!)
		inverse_dist *= inverse_dist;
		// avoid wall if withing plane avoid distance
		wallAvoidance += PLANE_SOFTNESS * inverse_dist * o.normal   * (float)(dist < PLANE_AVOID_DISTANCE);
	}


	if (nrPrey > 0 && bIsPrey > 0) {
		avgPreyPos /= nrPrey;
		avgPreyVel /= nrPrey;
		separation /= nrPrey;
		cohesion += avgPreyPos - b.position;
		alignment += avgPreyVel - b.velocity;
		if (nrPredators > 0) {
			avgPredatorPos /= nrPredators; 
			avoidance += b.position - avgPredatorPos;
		}
	}


	float acc = AVAILABLE_ACCELERATION;

	tapAcceleration(acc, wallAvoidance, newVel);
	tapAcceleration(acc, avoidance*12.f, newVel);
	tapAcceleration(acc, repellentForce*20.f, newVel);
	tapAcceleration(acc, separation*8.f, newVel);
	tapAcceleration(acc, cohesion*0.1f, newVel);
	tapAcceleration(acc, alignment, newVel);


	// Predator behavior
	if (bIsPredator > 0) {
		if (preyFound) {
			//newVel += glm::normalize(closestPreyPos - b.position) * PRED_ATTRACT_FACTOR * bIsPredator;
			newVel += glm::normalize(avgPreyPos - b.position) * PRED_ATTRACT_FACTOR * bIsPredator;
		}
		else {
			newVel = oldVel;
		}
	}


	// WEIGHTED AVERAGE
	/*

	//Avoid walls
	for (int m = 0; m < 6; m++) {
	ObstaclePlane o = walls[m];
	glm::vec3 v = b.position - o.point;
	float distance = PLANE_SOFTNESS / (glm::dot(v, o.normal) + epsilon);
	newVel += o.normal / (length(o.normal) + epsilon) * distance - b.velocity;
	}


	// Prey behavior
	if (nrPrey > 0 && bIsPrey > 0) {
	avgPreyPos /= nrPrey;
	avgPreyVel /= nrPrey;
	newVel += glm::normalize(avgPreyPos - b.position) * COHESION_FACTOR;
	newVel += glm::normalize(avgPreyVel - b.velocity) * ALIGN_FACTOR;
	newVel += separation;
	if (nrPredators > 0) {
	avgPredatorPos /= nrPredators;
	newVel += glm::normalize(b.position - avgPredatorPos) * AVOID_FACTOR;
	}
	}


	// Predator behavior
	if (bIsPredator > 0) {
	if (preyFound) {
	//newVel += glm::normalize(closestPreyPos - b.position) * PRED_ATTRACT_FACTOR * bIsPredator;
	newVel += glm::normalize(avgPreyPos - b.position) * PRED_ATTRACT_FACTOR * bIsPredator;
	}
	else {
	newVel = oldVel;
	}
	}
	*/




	// Limit acceleration
	glm::vec3 acceleration = newVel - oldVel;

	float accelerationMag = glm::clamp(glm::length(acceleration), 0.f, MAX_ACCELERATION * (1 + 0.7f*(nrPredators > 0)*bIsPrey));

	if (accelerationMag > epsilon) {
		newVel = oldVel + glm::normalize(acceleration)*accelerationMag;

		//Limit velocity
		float speed = glm::clamp(glm::length(newVel), MIN_SPEED, MAX_SPEED * (1 + (nrPredators > 0)*bIsPrey));
		newVel = glm::normalize(newVel) * speed;
	}
	else {
		newVel = oldVel;
	}

	// Update position 
	glm::vec3 newPos = b.position + newVel;//

	// TODO: Right now we wrap the boids around a cube
	// TODO: This is just a quickfix. Just assigning MAX_COORD is not exactly accurate
	// Also, is a modulus operation possibly cheaper?
	newPos.x = newPos.x < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.x;
	newPos.y = newPos.y < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.y;
	newPos.z = newPos.z < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.z;

	newPos.x = newPos.x > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.x;
	newPos.y = newPos.y > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.y;
	newPos.z = newPos.z > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.z;

	boidsUpdated[idx].position = newPos;
	boidsUpdated[idx].velocity = newVel;
	boidsUpdated[idx].status = b.status;
}

#ifdef NAIVE_VERSION
__global__ void computeVelocitiesNaive(Boid boids[], int cellStarts[], int cellEnds[], uint64_t cellIDs[]
	, int nrBoids, Boid boidsUpdated[], ObstaclePlane walls[]) {

	int idx = threadIdx.x + (blockIdx.x * blockDim.x);
	if (idx >= nrBoids) return;
	int neighbourCount = 0;

	Boid b = boids[idx]; // current boid whose neighbours we're checking
						 // initialize default values for each rule
						 // Decide which cell current boid is in
	glm::vec3 cell = getCell(b.position);

	float bIsPredator = (bool)(b.status & PREDATOR_FLAG);
	float bIsPrey = (bool)!(b.status & PREDATOR_FLAG);

	glm::vec3 oldVel = b.velocity;
	glm::vec3 avgPos = glm::vec3(0);
	glm::vec3 avgVel = b.velocity;

	glm::vec3 separation = glm::vec3(0);
	glm::vec3 cohesion = glm::vec3(0);
	glm::vec3 alignment = glm::vec3(0);

	glm::vec3 newVel = glm::vec3(0);

	bool preyFound = false;

	glm::vec3 planeforce = glm::vec3(0.0);
	glm::vec3 closestPreyPos = glm::vec3(0.0);
	float epsilon = 0.f;

	// Start checking all 27 neighbouring cells
	// TODO: find out a clever way to iterate over cells in order of the morton code to get 
	// more coherent memory accesses
	for (int l = 0; l <= nrBoids; l++) {
		Boid n = boids[l];
		Boid neighbour = boids[l];

		glm::vec3 delta = n.position - b.position;
		float distance = glm::length(delta);

		// exclude neighbours that are outside boid's scope 
		float validNeighbour = (distance > epsilon && distance < BOID_SCOPE);

		neighbourCount += validNeighbour;

		avgPos += validNeighbour * n.position;
		avgVel += validNeighbour * n.velocity;

		float nIsPredator = (bool)(n.status & PREDATOR_FLAG);
		float nIsPrey = (bool)(!n.status & PREDATOR_FLAG);

		// Separation and predator avoidance
		if (validNeighbour > 0) {
			separation -= bIsPrey * nIsPrey * glm::normalize(delta) * SEPARATION_FACTOR;
			separation -= nIsPredator * glm::normalize(delta) * AVOID_FACTOR;
		}
		// Hunt 
		if (nIsPrey > 0 && (distance < glm::distance(b.position, closestPreyPos) || !preyFound)) {
			closestPreyPos = n.position;
			preyFound = true;
		}
	}


	//Avoid walls
	for (int m = 0; m < 6; m++) {
		ObstaclePlane o = walls[m];
		glm::vec3 v = b.position - o.point;
		float distance = PLANE_SOFTNESS / (glm::dot(v, o.normal) + epsilon);
		planeforce += o.normal / (length(o.normal) + epsilon) * distance - b.velocity;
	}

	// Cohesion alignment and hunt
	if (neighbourCount > 0) {
		avgPos /= neighbourCount;
		avgVel /= neighbourCount;
		newVel += glm::normalize(avgPos - b.position) * COHESION_FACTOR * bIsPrey;
		newVel += glm::normalize(avgVel - b.velocity) * ALIGN_FACTOR * bIsPrey;
		newVel += separation;
	}

	// Hunt
	if (preyFound) {
		b.velocity += glm::normalize(closestPreyPos - b.position) * PRED_ATTRACT_FACTOR * bIsPredator;
	}
	// Limit acceleration
	glm::vec3 acceleration = newVel - oldVel;
	float accelerationMag = glm::clamp(glm::length(acceleration), 0.f, MAX_ACCELERATION);
	if (accelerationMag > epsilon) {
		newVel = oldVel + glm::normalize(acceleration)*accelerationMag;

		//Limit velocity
		float speed = glm::clamp(glm::length(newVel), MIN_SPEED, MAX_SPEED);
		newVel = glm::normalize(newVel) * speed;
	}
	else {
		// printf("Booo resetting velocity!!!!!!!!!!!!!!!!!!\n");
		newVel = oldVel;
	}

	// Update position 
	glm::vec3 newPos = b.position + newVel;

	// TODO: Right now we wrap the boids around a cube
	// TODO: This is just a quickfix. Just assigning MAX_COORD is not exactly accurate
	// Also, is a modulus operation possibly cheaper?
	newPos.x = newPos.x < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.x;
	newPos.y = newPos.y < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.y;
	newPos.z = newPos.z < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.z;

	newPos.x = newPos.x > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.x;
	newPos.y = newPos.y > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.y;
	newPos.z = newPos.z > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.z;

	boidsUpdated[idx].position = newPos;
	boidsUpdated[idx].velocity = newVel;
}
#endif

// Sets all the cell start/end indices to -1, so no old values is left
// TODO: only reset the ones that actually has had boids in it?
__global__ void resetCellRanges(int cellStarts[], int cellEnds[], int nrCells) {
	int i = threadIdx.x + (blockIdx.x * blockDim.x);
	if (i < nrCells) {
		cellStarts[i] = -1;
		cellEnds[i] = -1;
	}
}

// Stores the Morton code/Z-order value for each boid, based on the coordinates of the 
// cell which the boid currently is in
__global__ void calculateCellID(int n, uint64_t cellIDs[], Boid b[], int nrBoids) {
	int i = threadIdx.x + (blockIdx.x * blockDim.x);
	glm::vec3 cell = getCell(b[i].position);
#ifdef BIT_CONCATENATION
	cellIDs[i] = bitConcatenation((int)cell.x, (int)cell.y, (int)cell.z);
#else
	cellIDs[i] = bitInterlaceMagic((int)cell.x, (int)cell.y, (int)cell.z);
#endif
}

// After boid IDs are sorted the array with the actual boid structs are sorted accordingly with this function
__global__ void rearrangeBoids(int boidIDs[], Boid boids[], Boid boidsAlt[], int nrBoids) {
	int i = threadIdx.x + (blockIdx.x * blockDim.x);
	if (i >= nrBoids) return;
	boidsAlt[i] = boids[boidIDs[i]]; // copy over boids to the boidsAlt array, which in the end will be sorted
}

__global__ void prepareBoidRenderKernel(Boid* boids, glm::vec3* renderBoids, glm::mat4 projection, glm::mat4 view, float t) {
	int i = threadIdx.x + (blockIdx.x * blockDim.x);
	int j = i * 54;
	if (i >= NR_BOIDS) return;
	Boid b = boids[i];

	// float wingHeight = cosf(t + length(b.velocity));
	float wingHeight = cosf(t/5.f + b.position.x) + -0.6f;


	// one vector for each vertex

	glm::vec3 p1(0.0f, 1.5f, -1.73205080757f);

	glm::vec3 p0(-1.0f, -1.0f, wingHeight);
	glm::vec3 p2(0.0f, -1.0f, -1.73205080757f);
	glm::vec3 p3(1.0f, -1.0f, wingHeight);

	glm::vec3 p4(0.0f, -1.0f, -1.33205080757f); // Last point is less away than back point p3

	// dirty quickfix
	glm::vec3 color(198.f / 255.f, 231.f / 255.f, 1.0f);

	// create model matrix from agent position
	glm::mat4 model = glm::mat4(1.0f);
	model = glm::translate(model, b.position);
	glm::vec3 v = glm::vec3(b.velocity.z, 0, -b.velocity.x);
	float angle = acosf(b.velocity.y / glm::length(b.velocity)); // acosf is single precision == faster
	model = glm::rotate(model, angle, v);

	if (b.status & PREDATOR_FLAG) {
		color = glm::vec3(133.f / 255.f, 30.f / 255.f, 62.f / 255.f);
		model = glm::scale(model, glm::vec3(3.0f));
	}

	glm::vec3 v0 = view * model * glm::vec4(p0, 1.0f);
	glm::vec3 v1 = view * model * glm::vec4(p1, 1.0f);
	glm::vec3 v2 = view * model * glm::vec4(p2, 1.0f);
	glm::vec3 v3 = view * model * glm::vec4(p3, 1.0f);
	glm::vec3 v4 = view * model * glm::vec4(p4, 1.0f);

	glm::vec3 n0 = glm::mat3(glm::transpose(glm::inverse(model))) * glm::cross(p2 - p0, p1 - p0); // vänster vinge upp
	glm::vec3 n1 = glm::mat3(glm::transpose(glm::inverse(model))) * glm::cross(p1 - p0, p4 - p0); // vänster vinge ner 
	glm::vec3 n2 = glm::mat3(glm::transpose(glm::inverse(model))) * glm::cross(p3 - p2, p1 - p2); // höger vinge upp
	glm::vec3 n3 = glm::mat3(glm::transpose(glm::inverse(model))) * glm::cross(p1 - p4, p3 - p4); // höger vinge ner
	glm::vec3 n4 = glm::mat3(glm::transpose(glm::inverse(model))) * glm::cross(p4 - p0, p2 - p0); // vänster skinka
	glm::vec3 n5 = glm::mat3(glm::transpose(glm::inverse(model))) * glm::cross(p2 - p3, p4 - p3); // höger skinka

	glm::vec3 red(1.0f, 0.0f, 0.0f);
	glm::vec3 green(0.0f, 1.0f, 0.0f);
	glm::vec3 blue(0.0f, 0.0f, 1.0f);
	glm::vec3 white(1.0f, 1.0f, 1.0f);
	glm::vec3 black(0.0f, 0.0f, 0.0f);

	// vänster vinge upp
	renderBoids[j + 0] = v0;
	renderBoids[j + 1] = color;
	renderBoids[j + 2] = n0;
	renderBoids[j + 3] = v1;
	renderBoids[j + 4] = color;
	renderBoids[j + 5] = n0;
	renderBoids[j + 6] = v2;
	renderBoids[j + 7] = color;
	renderBoids[j + 8] = n0;

	// vänster vinge ner
	renderBoids[j + 9] = v0;
	renderBoids[j + 10] = color;
	renderBoids[j + 11] = n1;
	renderBoids[j + 12] = v4;
	renderBoids[j + 13] = color;
	renderBoids[j + 14] = n1;
	renderBoids[j + 15] = v1;
	renderBoids[j + 16] = color;
	renderBoids[j + 17] = n1;

	// höger vinge upp
	renderBoids[j + 18] = v1;
	renderBoids[j + 19] = color;
	renderBoids[j + 20] = n2;
	renderBoids[j + 21] = v3;
	renderBoids[j + 22] = color;
	renderBoids[j + 23] = n2;
	renderBoids[j + 24] = v2;
	renderBoids[j + 25] = color;
	renderBoids[j + 26] = n2;

	// höger vinge ner
	renderBoids[j + 27] = v1;
	renderBoids[j + 28] = color;
	renderBoids[j + 29] = n3;
	renderBoids[j + 30] = v4;
	renderBoids[j + 31] = color;
	renderBoids[j + 32] = n3;
	renderBoids[j + 33] = v3;
	renderBoids[j + 34] = color;
	renderBoids[j + 35] = n3;

	// vänster skinka
	renderBoids[j + 36] = v3;
	renderBoids[j + 37] = color;
	renderBoids[j + 38] = n4;
	renderBoids[j + 39] = v4;
	renderBoids[j + 40] = color;
	renderBoids[j + 41] = n4;
	renderBoids[j + 42] = v2;
	renderBoids[j + 43] = color;
	renderBoids[j + 44] = n4;

	// höger skinka
	renderBoids[j + 45] = v2;
	renderBoids[j + 46] = color;
	renderBoids[j + 47] = n5;
	renderBoids[j + 48] = v4;
	renderBoids[j + 49] = color;
	renderBoids[j + 50] = n5;
	renderBoids[j + 51] = v0;
	renderBoids[j + 52] = color;
	renderBoids[j + 53] = n5;
}

void prepareBoidRender(Boid* boids, glm::vec3* renderBoids, glm::mat4 projection, glm::mat4 view) {
	prepareBoidRenderKernel << < numBlocksBoids, blockSize >> > (boids, renderBoids, projection, view, t);
}

void printCUDAInfo() {
	int nDevices;

	cudaGetDeviceCount(&nDevices);
	for (int i = 0; i < nDevices; i++) {
		cudaDeviceProp prop;
		cudaGetDeviceProperties(&prop, i);
		printf("Device Number: %d\n", i);
		printf("  Device name: %s\n", prop.name);
		printf("  Memory Clock Rate (KHz): %d\n",
			prop.memoryClockRate);
		printf("  Memory Bus Width (bits): %d\n",
			prop.memoryBusWidth);
		printf("  Peak Memory Bandwidth (GB/s): %f\n\n",
			2.0*prop.memoryClockRate*(prop.memoryBusWidth / 8) / 1.0e6);
	}
}

void printCUDAError() {
	cudaError_t errSync = cudaGetLastError();
	cudaError_t errAsync = cudaDeviceSynchronize();
	if (errSync != cudaSuccess)
		printf("Sync kernel error: %s\n", cudaGetErrorString(errSync));
	if (errAsync != cudaSuccess)
		printf("Async kernel error: %s\n", cudaGetErrorString(errAsync));
}

__host__ Boid** initBoidsOnGPU(Boid* boidsArr) {
	printCUDAInfo();
	boids = boidsArr; // TODO: clean up all "boids" pointers

	gpuErrchk(cudaMallocManaged((void**)&walls, sizeof(ObstaclePlane) * 6));
	// Allocate memory for the cell index arrays
	gpuErrchk(cudaMallocManaged((void**)&cellStartIndex, sizeof(int) * NR_CELLS));
	gpuErrchk(cudaMallocManaged((void**)&cellEndIndex, sizeof(int) * NR_CELLS));
	// Allocate memory for the temp storage of new velocities
	gpuErrchk(cudaMallocManaged((void**)&newVelocities, sizeof(glm::vec3) * NR_BOIDS));
	// Allocate memory for the boids
	gpuErrchk(cudaMallocManaged((void**)&boidsAlt, sizeof(Boid) * NR_BOIDS));
	gpuErrchk(cudaMallocManaged((void**)&boids, sizeof(Boid) * NR_BOIDS));
	// Allocate memory for the buffer arrays
	gpuErrchk(cudaMallocManaged((void**)&cellIDs, sizeof(*cellIDs) * NR_BOIDS));
	gpuErrchk(cudaMallocManaged((void**)&cellIDsAlt, sizeof(*cellIDsAlt) * NR_BOIDS));
	gpuErrchk(cudaMallocManaged((void**)&boidIDs, sizeof(*boids) * NR_BOIDS));
	gpuErrchk(cudaMallocManaged((void**)&boidIDsAlt, sizeof(*boidIDsAlt) * NR_BOIDS));

	cellIDsBuf = DoubleBuffer<uint64_t>(cellIDs, cellIDsAlt);
	boidIDsBuf = DoubleBuffer<int>(boidIDs, boidIDsAlt);

	float max = MAX_COORD;
	float min = 0;
	// höger vägg
	walls[0] = ObstaclePlane(max, max/2.f, max / 2.f, -1, 0, 0);
	// vänster vägg
	walls[1] = ObstaclePlane(min, max / 2.f, max / 2.f, 1, 0, 0);
	// tak
	walls[2] = ObstaclePlane(max / 2.f, max, max / 2.f, 0, -1, 0);
	// golv
	walls[3] = ObstaclePlane(max / 2.f, min, max / 2.f, 0, 1, 0);
	// vägg framför
	walls[4] = ObstaclePlane(max / 2.f, max / 2.f, max, 0, 0, -1);
	// vägg bakom
	walls[5] = ObstaclePlane(max / 2.f, max / 2.f, min, 0, 0, 1);



	return &boids;
}

__host__ void deinitBoidsOnGPU() {
	// Free memory
	cudaFree(cellStartIndex);
	cudaFree(cellEndIndex);
	cudaFree(cellIDsBuf.d_buffers[0]);
	cudaFree(cellIDsBuf.d_buffers[1]);
	cudaFree(boidIDsBuf.d_buffers[0]);
	cudaFree(boidIDsBuf.d_buffers[1]);
	cudaFree(newVelocities);
	cudaFree(boids);
	cudaFree(boidsAlt);
}


void cudaGraphicsGLRegisterBufferWrapper(struct cudaGraphicsResource** positionsVBO_CUDA, unsigned int positionsVBO) {
	// TODO: cudaGraphicsRegisterFlagsWriteDiscard may be a useful flag here!
	cudaGraphicsGLRegisterBuffer(positionsVBO_CUDA, positionsVBO, cudaGraphicsRegisterFlagsWriteDiscard);
}

/* After calling this, you are free to execute CUDA kernels on the buffer! */




void mapBufferObjectCuda(struct cudaGraphicsResource** positionsVBO_CUDA, size_t* num_bytes, glm::vec3** positions) {
	cudaGraphicsMapResources(1, positionsVBO_CUDA, 0);
	cudaGraphicsResourceGetMappedPointer((void**)positions, num_bytes, *positionsVBO_CUDA);
}

void cudaGraphicsUnmapResourcesWrapper(struct cudaGraphicsResource** positionsVBO_CUDA) {
	cudaGraphicsUnmapResources(1, positionsVBO_CUDA, 0);
}

void cudaGraphicsUnregisterResourceWrapper(struct cudaGraphicsResource* positionsVBO_CUDA) {
	cudaGraphicsUnregisterResource(positionsVBO_CUDA);
}

void cudaSetDeviceWrapper(int n) {
	cudaSetDevice(n);
}

void step() {
	
	t += 1.0f;
	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);

#ifndef NAIVE_VERSION
#ifdef TIMING
	cudaEventRecord(start);
#endif 

	// Initialize boid id's
	initBoidIDs << < numBlocksBoids, blockSize >> > (boidIDsBuf.Current(), NR_BOIDS);

#ifdef TIMING
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	float milliseconds = 0;
	cudaEventElapsedTime(&milliseconds, start, stop);
	printf("initBoids: %f\n", milliseconds);
#endif 


#ifdef TIMING
	cudaEventRecord(start);
#endif 

	// Calculate cell IDs for every boid
	calculateCellID << < numBlocksBoids, blockSize >> > (NR_BOIDS, cellIDsBuf.Current(), boids, NR_BOIDS);

#ifdef TIMING
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&milliseconds, start, stop);
	printf("calculateCellID: %f\n", milliseconds);
#endif 


#ifdef TIMING
	cudaEventRecord(start);
#endif 

	// reset cell ranges
	resetCellRanges << < numBlocksCells, blockSize >> > (cellStartIndex, cellEndIndex, NR_CELLS);

#ifdef TIMING
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&milliseconds, start, stop);
	printf("resetCellRanges: %f\n", milliseconds);
#endif 

	// Determine temporary device storage requirements
	void     *d_temp_storage = NULL;
	size_t   temp_storage_bytes = 0;

#ifdef TIMING
	cudaEventRecord(start);
#endif 

	// Determine temporary storage need
	cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, cellIDsBuf, boidIDsBuf, NR_BOIDS);

	// Allocate temporary storage
	// TODO: cudaMalloc is expensive, is it possible to do this particular allocation only once and reuse it? 
	cudaMalloc(&d_temp_storage, temp_storage_bytes);

	// Run sorting operation
	cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, cellIDsBuf, boidIDsBuf, NR_BOIDS);

	cudaFree(d_temp_storage);

#ifdef TIMING
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&milliseconds, start, stop);
	printf("Sorting: %f\n", milliseconds);
#endif 

#ifdef TIMING
	cudaEventRecord(start);
#endif 

	// Rearrange the actual boids based on the sorted boidIDs
	rearrangeBoids << < numBlocksBoids, blockSize >> > (boidIDsBuf.Current(), boids, boidsAlt, NR_BOIDS);
	// After rearranging the boids, we now work on the boidsAlt

#ifdef TIMING
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&milliseconds, start, stop);
	printf("rearrangeBoids: %f\n", milliseconds);
#endif 


#ifdef TIMING
	cudaEventRecord(start);
#endif 

	// Check were cellID changes occurs in the sorted boids array
	detectCellIndexChange << < numBlocksBoids, blockSize >> > (cellStartIndex, cellEndIndex, cellIDsBuf.Current(), NR_BOIDS);

#ifdef TIMING
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&milliseconds, start, stop);
	printf("detectCellIndexChange: %f\n", milliseconds);
#endif 

#ifdef TIMING
	cudaEventRecord(start);
#endif 

	// Update boid velocities based on the rules
	computeVelocities << < numBlocksBoids, blockSize >> > (boidsAlt, cellStartIndex, cellEndIndex, cellIDsBuf.Current(), NR_BOIDS, boids, walls, cameraPos, cameraDir, isLaserActive);

#ifdef TIMING
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&milliseconds, start, stop);
	printf("computeVelocities: %f\n\n", milliseconds);
#endif 

#endif // END UNIFORM VERSION

#ifdef NAIVE_VERSION
	computeVelocitiesNaive << < numBlocksBoids, blockSize >> > (boids, cellStartIndex, cellEndIndex, cellIDsBuf.Current(), NR_BOIDS, boidsAlt, walls);
	std::swap(boids, boidsAlt);
#endif

	// Swap the boids array pointer, so 'boids' now points to a sorted array
	cudaDeviceSynchronize(); // TODO: is this call necessary?

}
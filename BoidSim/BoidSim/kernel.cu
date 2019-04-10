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

// includes, cuda
#include <windows.h>
//#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

/* 
Compile on Linux machines in NC:  
/chalmers/sw/sup64/cuda_toolkit-9.0.176.4/bin/nvcc --dont-use-profile -ldir /chalmers/sw/sup64/cuda_toolkit-9.0.176.4/nvvm/libdevice/ -I /chalmers/sw/sup64/cuda_toolkit-9.0.176.4/include -m64 -L /chalmers/sw/sup64/cuda_toolkit-9.0.176.4/lib64 ~/kernel.cu

You might need this before compiling: 
PATH=$PATH:/chalmers/sw/sup64/cuda_toolkit-9.0.176.4/nvvm/bin
PATH=$PATH:/chalmers/sw/sup64/cuda_toolkit-9.0.176.4/bin

There are many optimization flags. See here:
https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html#precision-related-compiler-flags

*/ 

/* A useful macro for displaying CUDA errors */ 
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=false)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

using namespace cub;

// Boid attributes
#define MAX_SPEED 30.0f
#define MIN_SPEED 20.0f // TODO 

// #define DEBUG

// These arrays hold the boids
extern Boid* boids;
Boid* boidsSorted = NULL;

// These arrays hold the (Z-order/morton encoded) cell ids
uint64_t* boidCellIDs = NULL;
uint64_t* boidCellIDsAlt = NULL;

// Array with all the boids. boidsSorted is a alternate array needed for the radixSort
int* boidIDs = NULL;
int* boidIDsAlt = NULL;

// Doublebuffers containing boidIDs and cellIDs, these are used by the radix sort function
DoubleBuffer<uint64_t> boidCellIDsBuf;
DoubleBuffer<int> boidIDsBuf;

// Calculate the maximum value of Morton encoded (Z-ordered) cell ids
#define shiftBitK(x, k) (int) ((x&(1<<k)) << k*2+2 | (x&(1<<k)) << k*2+1 | (x&(1<<k)) << k*2)
const int MAX_CELL_INDEX = (int) MAX_COORD/CELL_SIZE;
const int NR_CELLS = shiftBitK(MAX_CELL_INDEX, 10) 
                     |shiftBitK(MAX_CELL_INDEX, 9)
                     |shiftBitK(MAX_CELL_INDEX, 8)
                     |shiftBitK(MAX_CELL_INDEX, 7)
                     |shiftBitK(MAX_CELL_INDEX, 6)
                     |shiftBitK(MAX_CELL_INDEX, 5)
                     |shiftBitK(MAX_CELL_INDEX, 4)
                     |shiftBitK(MAX_CELL_INDEX, 3)
                     |shiftBitK(MAX_CELL_INDEX, 2)
                     |shiftBitK(MAX_CELL_INDEX, 1)
                     |shiftBitK(MAX_CELL_INDEX, 0);

// These parameters are used by the CUDA functions
int blockSize = 256;
int numBlocksBoids = (NR_BOIDS + blockSize - 1) / blockSize;
int numBlocksCells = (NR_CELLS + blockSize - 1) / blockSize;

// A tempory storage for new velocities allows parallel processing of the boids velocities 
glm::vec3* newVelocities;

// These arrays hold the start and end indices for each cell which contains boids
int* cellStartIndex;
int* cellEndIndex;

// Get the cell based on the boids position
inline __device__ glm::vec3 getCell(glm::vec3 pos){
    return glm::floor(pos * (1.0f/CELL_SIZE));
}

// The boids are given an ID, which is the index it has in the intial boid array
__global__ void initBoidIDs(int BoidIDs[], int nrBoids){
    int i = (blockIdx.x * blockDim.x) + threadIdx.x; 
    if(i >= nrBoids) return; 
    BoidIDs[i] = i;
}

// Spreads out bits with two zeroes in between
// TODO: find the source and give credit
__device__ uint64_t spreadOutByThree(uint64_t i){
    i = (i | (i << 16)) & 0x030000FF;
    i = (i | (i <<  8)) & 0x0300F00F;
    i = (i | (i <<  4)) & 0x030C30C3;
    i = (i | (i <<  2)) & 0x09249249;
    return i;
}


// Hash cell coords to morton code with "magic numbers"
__device__ uint64_t bitInterlaceMagic(int x, int y, int z){
    return spreadOutByThree((uint64_t)x) | (spreadOutByThree((uint64_t)y) << 1) | (spreadOutByThree((uint64_t)z) << 2);
}

// This function is used when scanning the sorted boids cell-ids to see were cells starts and ends 
// dummyHostIndex is only for debugging on machines without device enabled
__global__ void detectCellIndexChange(int cellStarts[], int cellEnds[], uint64_t cellIDs[], int nrBoids){
    int i = (blockIdx.x * blockDim.x) + threadIdx.x; 
    if(i >= nrBoids) return; 
    
    int cellID = cellIDs[i];
    int nextCellID = cellIDs[i+1];
    #ifdef DEBUG
    printf("Checking index %d which has value %d \n", i, cellID);
    #endif
    // TODO: determine if these if/else statements causes thread branching -> worse performance
    if(i == 0){ 
        // This is the case for the first element in the boid array 
        cellStarts[cellID] = i; 
        #ifdef DEBUG
        printf("Cell start was detected: from %d \n", cellID);
        #endif
    } else if (i == nrBoids - 1){ 
        // This is the case for the last element in the boid array
        cellEnds[cellID] = i;
        #ifdef DEBUG
        printf("Cell end was detected: from %d \n", cellID);
        #endif
        return;
    } 
    if (cellID != nextCellID){
        // A change in cell index was detected!
        #ifdef DEBUG
        printf("A cell change was detected: from %d to %d \n", cellID, nextCellID);
        #endif
        cellStarts[nextCellID] = i + 1;
        cellEnds[cellID] = i;
    }
}

// Update boid with index n
// WARNING! VERY MUCH TODO: RIGHT NOW IT CHECKS OUTSIDE WORLD BOUNDARIES FOR BOIDS
// IN EDGE CELLS, THIS WILL CAUSE ARRAY OUT OF-BOUNDS EXCEPTIONS
__global__ void computeVelocities(Boid boids[], int cellStarts[], int cellEnds[], uint64_t cellIDs[]
                                , int nrBoids, glm::vec3 newVelocities[]){
    int i = threadIdx.x + (blockIdx.x * blockDim.x);
    if(i >= nrBoids) return;
    int neighbourCount = 0;
    Boid b = boids[i]; // current boid whose neighbours we're checking
    // initialize default values for each rule
    glm::vec3 alignment = b.velocity;
	glm::vec3 separation = glm::vec3(0.0);
    glm::vec3 cohesion = glm::vec3(0.0);
    // Decide which cell current boid is in
    glm::vec3 cell = getCell(b.position);
    // Start checking all 27 neighbouring cells
    // TODO: find out a clever way to iterate over cells in order of the morton code to get 
    // more coherent memory accesses
    for(int i = -1; i <= 1; i++){
        for(int j = -1; j <= 1; j++){
            for(int k = -1; k <= 1; k++){
                // calculate the (Morton encoded/Z-order) cell id based on cell coordinates
                uint64_t cellID = bitInterlaceMagic((int)cell.x + i, (int)cell.y + j, (int)cell.z + k);
                // TODO: decide wether the if-statement below causes thread branching 
                if (cellStarts[cellID] == -1) {
                    continue; // cell is empty if it's start index is unchanged since last reset
                }
                // Iterate over all boids in neighbouring cell
                for (int l = cellStarts[cellID]; l <= cellEnds[cellID]; l++){
                    float distance = glm::distance(b.position, boids[l].position);
                    Boid neighbour = boids[l];
                    // exclude neighbours that are outside boid's scope 
                    float validNeighbour = (neighbour.position != b.position && distance < BOID_SCOPE);
                    neighbourCount += validNeighbour;
                    // Apply rules. Factor "validNeighbour" is zero for non-valid neighbours
                    alignment += validNeighbour * neighbour.velocity * 4.0f/(distance + 0.0000001f); // + 0.0001 is for avoiding divide by zero
                    separation += validNeighbour * (b.position - neighbour.position) * 1.0f/(float)(distance*distance + 0.0000001f); // + 0.0001 is for avoiding divide by zero
                    cohesion += validNeighbour * neighbour.position;
                }
            }
        }
    }
    // Divide by number of neighbours to get average values
    alignment = alignment * (1.0f / (neighbourCount + 1));
    // TODO: This is a debug quickfix, should not be an if here because it causes thread branching
    if( neighbourCount != 0){
	    cohesion = cohesion * (1.0f / (neighbourCount + 0.0000000001f)) - b.position; // We need 0.0000000001 here to avoid divide by zero
    }
    separation = separation * (1.0f / (neighbourCount + 0.0000000001f));
    
    /*Update Velocity*/
    glm::vec3 newVel = alignment + 50.0f*separation + 0.9f*cohesion;
    float speed = glm::clamp(length(newVel), MIN_SPEED, MAX_SPEED); // limit speed

	/* Update Velocity */
    newVelocities[i] = 0.01f*speed*glm::normalize(newVel);
    #ifdef DEBUG
    printf("Boid %d has %d neighbours\n", i, neighbourCount);
    #endif
} 

// Adds the new velocity value to the boids position, and copies the new velocity into the boid struct
// TODO: maybe we should place boid pos/vel in separate arrays, that way we don't have to copy the new velocities
// just swap pointers between two velocity arrays? 
__global__ void updatePosAndVel(Boid boids[], glm::vec3 newVelocities[], int nrBoids){
    int i = threadIdx.x + (blockIdx.x * blockDim.x);
    if(i >= nrBoids) return;
    glm::vec3 newPos = boids[i].position + newVelocities[i];
    // TODO: Right now we wrap the boids around a cube
    // TODO: This is just a quickfix. Just assigning MAX_COORD is not exactly accurate
    // Also, is a modulus operation possibly cheaper?
    newPos.x = newPos.x < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.x;
    newPos.y = newPos.y < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.y;
    newPos.z = newPos.z < CELL_SIZE ? MAX_COORD - CELL_SIZE : newPos.z;

	newPos.x = newPos.x > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.x;
	newPos.y = newPos.y > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.y;
	newPos.z = newPos.z > MAX_COORD - CELL_SIZE ? CELL_SIZE : newPos.z;

    boids[i].position = newPos;
    boids[i].velocity = newVelocities[i];
}

// Sets all the cell start/end indices to -1, so no old values is left
// TODO: only reset the ones that actually has had boids in it?
__global__ void resetCellRanges(int cellStarts[], int cellEnds[], int nrCells){
    int i = threadIdx.x + (blockIdx.x * blockDim.x);
    if(i < nrCells){
        cellStarts[i] = -1;
        cellEnds[i] = -1;
    }
}

// Stores the Morton code/Z-order value for each boid, based on the coordinates of the 
// cell which the boid currently is in
__global__ void calculateBoidHash(int n, uint64_t currentHashArray[], Boid b[]){
    int index = threadIdx.x;
    int stride = blockDim.x;
    for (int i = index; i < n; i += stride){
        glm::vec3 cell = getCell(b[i].position);
        currentHashArray[i] = bitInterlaceMagic((int)cell.x, (int)cell.y, (int)cell.z);
    }
}

// After boid IDs are sorted the array with the actual boid structs are sorted accordingly with this function
__global__ void rearrangeBoids(int boidIDs[], Boid boids[], Boid boidsSorted[], int nrBoids){
    int i = threadIdx.x + (blockIdx.x * blockDim.x);
    if (i >= nrBoids) return;
    boidsSorted[i] = boids[boidIDs[i]]; // copy over boids to the boidsSorted array, which in the end will be sorted
}

void printCUDAError();

__global__ void prepareBoidRenderKernel(Boid* boids, glm::vec3* renderBoids, glm::mat4 projection, glm::mat4 view){
    int i = threadIdx.x + (blockIdx.x * blockDim.x);
    int j = i*3;
    if(i >= NR_BOIDS) return;
    Boid b = boids[i];
    
    // one vector for each vertex
	const glm::vec3 p1(-1.0f, -1.0f, 0.0f);
	const glm::vec3 p2(0.0f, 1.0f, 0.0f);
    const glm::vec3 p3(1.0f, -1.0f, 0.0f);
    
    // create model matrix from agent position
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, b.position);
    glm::vec3 v = glm::vec3(b.velocity.z, 0, -b.velocity.x);
    float angle = acosf(b.velocity.y / glm::length(b.velocity)); // acosf is single precision == faster
    model = glm::rotate(model, angle, v);
    
    // transform each vertex and add them to array
    renderBoids[j] = view * model * glm::vec4(p1, 1.0f);
    renderBoids[j+1] = view * model * glm::vec4(p2, 1.0f);
    renderBoids[j+2] = view * model * glm::vec4(p3, 1.0f); 
}

void prepareBoidRender(Boid* boids, glm::vec3* renderBoids, glm::mat4 projection, glm::mat4 view){
    prepareBoidRenderKernel <<< numBlocksBoids, blockSize >>> (boids, renderBoids, projection, view);
}

void printCUDAInfo(){
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
             2.0*prop.memoryClockRate*(prop.memoryBusWidth/8)/1.0e6);
    }
}

void printCUDAError(){
    cudaError_t errSync  = cudaGetLastError();
    cudaError_t errAsync = cudaDeviceSynchronize();
    if (errSync != cudaSuccess) 
        printf("Sync kernel error: %s\n", cudaGetErrorString(errSync));
    if (errAsync != cudaSuccess)
        printf("Async kernel error: %s\n", cudaGetErrorString(errAsync));
}

__host__ Boid** initBoidsOnGPU(Boid* boidsArr){
    printCUDAInfo();
    boids = boidsArr; // TODO: clean up all "boids" pointers

    // Allocate memory for the cell index arrays
    gpuErrchk( cudaMallocManaged((void**)&cellStartIndex, sizeof(int) * NR_CELLS) );
    gpuErrchk( cudaMallocManaged((void**)&cellEndIndex, sizeof(int) * NR_CELLS) );
    // Allocate memory for the temp storage of new velocities
    gpuErrchk( cudaMallocManaged((void**)&newVelocities, sizeof(glm::vec3) * NR_BOIDS) );
    // Allocate memory for the boids
    gpuErrchk( cudaMallocManaged((void**)&boids, sizeof(Boid) * NR_BOIDS) );
    gpuErrchk( cudaMallocManaged((void**)&boidsSorted, sizeof(Boid) * NR_BOIDS) );
    // Allocate memory for the buffer arrays
    gpuErrchk( cudaMallocManaged((void**)&boidCellIDs, sizeof(*boidCellIDs) * NR_BOIDS) );
    gpuErrchk( cudaMallocManaged((void**)&boidCellIDsAlt, sizeof(*boidCellIDsAlt) * NR_BOIDS) );
    gpuErrchk( cudaMallocManaged((void**)&boidIDs, sizeof(*boids) * NR_BOIDS) );
    gpuErrchk( cudaMallocManaged((void**)&boidIDsAlt, sizeof(*boidIDsAlt) * NR_BOIDS) );

    boidCellIDsBuf = DoubleBuffer<uint64_t>(boidCellIDs, boidCellIDsAlt);
    boidIDsBuf = DoubleBuffer<int>(boidIDs, boidIDsAlt);
    return &boids; 
}

__host__ void deinitBoidsOnGPU(){
    // Free memory
    cudaFree(cellStartIndex);
    cudaFree(cellEndIndex);
    cudaFree(boidCellIDsBuf.d_buffers[0]);
    cudaFree(boidCellIDsBuf.d_buffers[1]);
    cudaFree(boidIDsBuf.d_buffers[0]);
    cudaFree(boidIDsBuf.d_buffers[1]);
    cudaFree(newVelocities);
    cudaFree(boids);
    cudaFree(boidsSorted);
}


void cudaGraphicsGLRegisterBufferWrapper( struct cudaGraphicsResource** positionsVBO_CUDA, unsigned int positionsVBO){
    // TODO: cudaGraphicsRegisterFlagsWriteDiscard may be a useful flag here!
    cudaGraphicsGLRegisterBuffer( positionsVBO_CUDA, positionsVBO, cudaGraphicsMapFlagsNone );
}

/* After calling this, you are free to execute CUDA kernels on the buffer! */ 
void mapBufferObjectCuda( struct cudaGraphicsResource** positionsVBO_CUDA, size_t* num_bytes, glm::vec3** positions){
    cudaGraphicsMapResources(1, positionsVBO_CUDA, 0);
    cudaGraphicsResourceGetMappedPointer((void**)positions, num_bytes, *positionsVBO_CUDA);
}

void cudaGraphicsUnmapResourcesWrapper(struct cudaGraphicsResource** positionsVBO_CUDA){
    cudaGraphicsUnmapResources(1, positionsVBO_CUDA, 0);
}

void cudaGraphicsUnregisterResourceWrapper(struct cudaGraphicsResource* positionsVBO_CUDA){
    cudaGraphicsUnregisterResource(positionsVBO_CUDA);
}

void cudaSetDeviceWrapper(int n){
    cudaSetDevice(n);
}

void step(){
    // Initialize boid id's
    initBoidIDs <<< numBlocksBoids, blockSize >>> (boidIDsBuf.Current(), NR_BOIDS);
    
    // Calculate cell IDs for every boid
    calculateBoidHash <<< numBlocksBoids, blockSize >>> (NR_BOIDS, boidCellIDsBuf.Current(), boids);
    
    // reset cell ranges
    resetCellRanges <<< numBlocksCells, blockSize >>> (cellStartIndex, cellEndIndex, NR_CELLS);
    
    // Determine temporary device storage requirements
    void     *d_temp_storage = NULL;
    size_t   temp_storage_bytes = 0;
    
    // Determine temporary storage need
    cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, boidCellIDsBuf, boidIDsBuf, NR_BOIDS);

    // Allocate temporary storage
    // TODO: cudaMalloc is expensive, is it possible to do this particular allocation only once and reuse it? 
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    
    // Run sorting operation
    cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, boidCellIDsBuf, boidIDsBuf, NR_BOIDS);
    
    cudaFree(d_temp_storage);

    // Rearrange the actual boids based on the sorted boidIDs
    rearrangeBoids <<< numBlocksBoids, blockSize >>> (boidIDsBuf.Current(), boids, boidsSorted, NR_BOIDS);
    
    // Check were cellID changes occurs in the sorted boids array
    detectCellIndexChange <<< numBlocksBoids, blockSize >>> (cellStartIndex, cellEndIndex, boidCellIDsBuf.Current(), NR_BOIDS);

    // Update boid velocities based on the rules
    computeVelocities <<< numBlocksBoids, blockSize >>> (boidsSorted, cellStartIndex, cellEndIndex, boidCellIDsBuf.Current(), NR_BOIDS, newVelocities);
    
    // Copy boid velocities from temporary velocity storage to boid array
    updatePosAndVel <<< numBlocksBoids, blockSize >>> (boidsSorted, newVelocities, NR_BOIDS);

    // Swap the boids array pointer, so 'boids' now points to a sorted array
    cudaDeviceSynchronize(); // TODO: is this call necessary?
    std::swap(boids, boidsSorted);
    
}
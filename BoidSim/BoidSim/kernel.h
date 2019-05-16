#ifndef kernel_h
#define kernel_h

#include "boid.h"

#define NR_BOIDS 500000

#define MAX_COORD 700.f // World boundaries, tested at 700.f
#define CELL_SIZE 10.f // The world is divided in cubic cells  
#define BOID_SCOPE 10.f // this is how far boids look for neighbours. Should always be == CELL_SIZE ?

// Boid attributes
#define PLANE_SOFTNESS 20.f
#define PLANE_AVOID_DISTANCE 40.f
#define DEATH_DISTANCE 3.5f
#define MAX_ACCELERATION 0.07f
#define MAX_ACCELERATION_PREDATOR 0.05f
#define SEPARATION_SOFTNESS 2.0f

#define SEPARATION_FACTOR .05f
#define ALIGN_FACTOR 0.3f
//#define COHESION_FACTOR 0.003f // this is the tested value
#define COHESION_FACTOR 0.1f // but this looks better
#define AVAILABLE_ACCELERATION 1.8f;

#define AVOID_FACTOR 2.0f
#define PRED_ATTRACT_FACTOR 2.0f

#define MIN_SPEED 0.2f
#define MAX_SPEED 0.3f


// Just a declaration, the actual struct is buried in the .obj file
struct cudaGraphicsResource;

Boid** initBoidsOnGPU(Boid*);
void deinitBoidsOnGPU(void);
void step();
void prepareBoidRender(Boid* boids, glm::vec3* renderBoids, glm::mat4 projection, glm::mat4 view);
void printCUDAError();


void cudaGraphicsGLRegisterBufferWrapper( struct cudaGraphicsResource** positionsVBO_CUDA , unsigned int positionsVBO);
void mapBufferObjectCuda( struct cudaGraphicsResource** positionsVBO_CUDA, size_t* num_bytes, glm::vec3** positions);
void cudaGraphicsUnmapResourcesWrapper(struct cudaGraphicsResource** positionsVBO_CUDA);
void cudaGraphicsUnregisterResourceWrapper(struct cudaGraphicsResource* positionsVBO_CUDA);

void cudaSetDeviceWrapper(int n);

#endif
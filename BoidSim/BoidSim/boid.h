#ifndef boid_h
#define boid_h

#include "glm/glm.hpp"
#include <vector>

#if defined(__CUDACC__) // NVCC
   #define MY_ALIGN(n) __align__(n)
#elif defined(__GNUC__) // GCC
  #define MY_ALIGN(n) alignas(n)
#elif defined(_MSC_VER) // MSVC
  #define MY_ALIGN(n) alignas(n)
#else
  #error "Please provide a definition for MY_ALIGN macro for your host compiler!"
#endif

// TODO: Testa olika align värden, kan påverka enligt CUDA docs 9.2.1.2. A Sequential but Misaligned Access Pattern
struct MY_ALIGN(8) Boid {
	glm::vec3 position, velocity;

	Boid()
		: position(rand() % 400 + 15, rand() % 400 + 15, rand() % 200 + 15), velocity(rand() % 10 + 15, rand() % 10 + 15, rand() % 10 + 15) { }
};

#endif
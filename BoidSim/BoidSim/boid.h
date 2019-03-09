#ifndef boid_h
#define boid_h

#include <glm/glm.hpp>
#include <vector>

struct Boid {
	glm::vec3 position, velocity;

	Boid()
		: position(rand() % 161 - 80, rand() % 61 - 30, rand() % 61 - 30), velocity(rand() % 61 - 30, rand() % 61 - 30, rand() % 61 - 30) { }
};

extern int nrBoids;
extern std::vector<Boid> boids;

#endif
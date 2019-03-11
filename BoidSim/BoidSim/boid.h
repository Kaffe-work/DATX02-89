#ifndef boid_h
#define boid_h

#include <glm/glm.hpp>
#include <vector>

struct Boid {
	glm::vec3 position, velocity;

	Boid()
		: position(rand() % 161 - 80, rand() % 161 - 80, 0), velocity(rand() % 161 - 80, rand() % 161 - 80, 0) { }
};

#endif